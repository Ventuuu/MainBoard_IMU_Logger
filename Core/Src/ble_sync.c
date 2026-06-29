#include "ble_sync.h"

#include <limits.h>
#include <string.h>

#include "bluetooth.h"
#include "main.h"
#include "SPI_NAND.h"

#define BLE_SYNC_RX_RING_BYTES 512U
#define BLE_SYNC_STATUS_PAYLOAD_BYTES 44U
#define BLE_SYNC_PAGE_BEGIN_PAYLOAD_BYTES 32U
#define BLE_SYNC_PAGE_END_PAYLOAD_BYTES 16U
#define BLE_SYNC_ACK_PAYLOAD_BYTES 8U
#define BLE_SYNC_NACK_PAYLOAD_BYTES 12U
#define BLE_SYNC_COMPLETE_PAYLOAD_BYTES 12U
#define BLE_SYNC_ABORT_PAYLOAD_BYTES 12U
#define BLE_SYNC_ERROR_PAYLOAD_BYTES 8U

_Static_assert((BLE_SYNC_RX_RING_BYTES & (BLE_SYNC_RX_RING_BYTES - 1U)) == 0U,
               "BLE RX ring size must be a power of two");
_Static_assert(BLE_SYNC_STATUS_PAYLOAD_BYTES <= BLE_SYNC_MAX_PAYLOAD_BYTES,
               "SYNC_STATUS payload exceeds frame capacity");

volatile BleSyncState ble_sync_state = BLE_SYNC_IDLE;
volatile uint8_t ble_sync_requested = 0U;
volatile uint8_t ble_sync_abort_requested = 0U;
volatile uint8_t ble_sync_active = 0U;

volatile uint32_t ble_sync_sessions_started = 0U;
volatile uint32_t ble_sync_sessions_completed = 0U;
volatile uint32_t ble_sync_sessions_aborted = 0U;

volatile uint32_t ble_sync_pages_planned = 0U;
volatile uint32_t ble_sync_pages_started = 0U;
volatile uint32_t ble_sync_pages_sent = 0U;
volatile uint32_t ble_sync_pages_acked = 0U;
volatile uint32_t ble_sync_pages_retransmitted = 0U;

volatile uint32_t ble_sync_frames_tx = 0U;
volatile uint32_t ble_sync_frames_rx = 0U;
volatile uint32_t ble_sync_frame_crc_errors = 0U;
volatile uint32_t ble_sync_frame_length_errors = 0U;
volatile uint32_t ble_sync_protocol_version_errors = 0U;
volatile uint32_t ble_sync_uart_errors = 0U;

volatile uint32_t ble_sync_ack_timeouts = 0U;
volatile uint32_t ble_sync_invalid_acks = 0U;
volatile uint32_t ble_sync_duplicate_acks = 0U;
volatile uint32_t ble_sync_nacks_received = 0U;

volatile uint32_t ble_sync_log_generation = 1U;
volatile uint8_t ble_sync_ack_valid = 0U;
volatile uint32_t ble_sync_acked_through_ram = 0U;
volatile uint32_t ble_sync_acked_through_persisted = 0U;
volatile uint32_t ble_sync_high_watermark = 0U;
volatile uint32_t ble_sync_high_watermark_physical_page = UINT32_MAX;
volatile uint32_t ble_sync_current_page_sequence = 0U;
volatile uint32_t ble_sync_current_physical_page = UINT32_MAX;
volatile uint32_t ble_sync_current_retry_count = 0U;

volatile uint16_t ble_sync_metadata_block_a = UINT16_MAX;
volatile uint16_t ble_sync_metadata_block_b = UINT16_MAX;
volatile uint32_t ble_sync_metadata_records_recovered = 0U;
volatile uint32_t ble_sync_metadata_records_written = 0U;
volatile uint32_t ble_sync_metadata_crc_errors = 0U;
volatile uint32_t ble_sync_metadata_write_errors = 0U;
volatile uint32_t ble_sync_metadata_erase_errors = 0U;

volatile uint32_t ble_sync_last_error = BLE_SYNC_ERROR_NONE;
volatile uint32_t ble_sync_last_abort_reason = BLE_SYNC_ABORT_NONE;
volatile uint32_t ble_sync_session_start_ms = 0U;
volatile uint32_t ble_sync_session_duration_ms = 0U;
volatile uint32_t ble_sync_protocol_self_test_failures = 0U;
volatile BleSyncLatestDiagnostics ble_sync_latest;

typedef struct
{
    NandLogger *logger;
    BleSyncFrameParser parser;
    uint16_t tx_message_sequence;

    uint8_t metadata_ready;
    uint8_t metadata_active_slot;
    uint8_t metadata_next_page;
    uint32_t metadata_sequence;
    uint8_t persisted_ack_valid;
    uint32_t acked_pages_since_persist;

    uint8_t prepare_started;
    uint32_t snapshot_used_pages;
    uint32_t snapshot_cursor;
    uint8_t snapshot_sequence_seen;
    uint32_t snapshot_last_sequence;
    uint8_t high_watermark_valid;
    uint32_t oldest_available_sequence;
    uint32_t newest_available_sequence;
    uint32_t first_sequence_to_send;
    uint32_t total_unsynced_pages;

    uint32_t find_cursor;
    uint8_t find_sequence_seen;
    uint32_t find_last_sequence;
    uint32_t current_logical_page;
    LogPageHeader current_header;
    uint32_t current_logical_bytes;
    uint32_t current_page_crc32;
    uint32_t current_data_offset;
    uint8_t current_page_sent;

    uint32_t start_deadline_ms;
    uint32_t ack_deadline_ms;
    uint8_t sync_start_received;
    uint8_t pending_ack;
    uint32_t pending_ack_generation;
    uint32_t pending_ack_sequence;
    uint8_t pending_nack;
    uint32_t pending_nack_generation;
    uint32_t pending_nack_sequence;
    uint8_t pending_nack_reason;
    uint8_t pending_error;
    uint16_t pending_error_code;
    uint32_t pending_error_detail;
    BleSyncAbortReason abort_reason;
} BleSyncContext;

static BleSyncContext ble_sync_context;
static uint8_t ble_sync_tx_frame[BLE_SYNC_MAX_FRAME_BYTES];
static uint8_t ble_sync_page_buffer[NAND_PAGE_SIZE_BYTES];
static uint8_t ble_sync_metadata_page_buffer[NAND_PAGE_SIZE_BYTES];

static volatile uint8_t ble_sync_uart_rx_byte;
static volatile uint8_t ble_sync_rx_ring[BLE_SYNC_RX_RING_BYTES];
static volatile uint16_t ble_sync_rx_head = 0U;
static volatile uint16_t ble_sync_rx_tail = 0U;
static volatile uint8_t ble_sync_uart_fault_pending = 0U;

extern UART_HandleTypeDef huart3;

static void ble_sync_put_u16_le(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void ble_sync_put_u32_le(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8U) & 0xFFU);
    dst[2] = (uint8_t)((value >> 16U) & 0xFFU);
    dst[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static uint32_t ble_sync_get_u32_le(const uint8_t *src)
{
    return ((uint32_t)src[0]) |
           ((uint32_t)src[1] << 8U) |
           ((uint32_t)src[2] << 16U) |
           ((uint32_t)src[3] << 24U);
}

static uint8_t ble_sync_deadline_reached(uint32_t now_ms,
                                         uint32_t deadline_ms)
{
    return ((int32_t)(now_ms - deadline_ms) >= 0) ? 1U : 0U;
}

static uint32_t ble_sync_next_nonzero_sequence(uint32_t value)
{
    value++;
    return (value == 0U) ? 1U : value;
}

static read_address_t ble_sync_metadata_address(uint8_t slot, uint8_t page)
{
    read_address_t address = {0};

    address.block = (slot == 0U) ?
                    ble_sync_metadata_block_a : ble_sync_metadata_block_b;
    address.page = page;
    address.dummy = 0U;
    return address;
}

static uint8_t ble_sync_buffer_is_erased(const uint8_t *data, uint32_t length)
{
    for (uint32_t i = 0U; i < length; i++)
    {
        if (data[i] != 0xFFU)
        {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t ble_sync_metadata_tail_is_erased(const uint8_t *page)
{
    return ble_sync_buffer_is_erased(
            &page[sizeof(BleSyncMetadataV1)],
            NAND_PAGE_SIZE_BYTES - sizeof(BleSyncMetadataV1));
}

static int ble_sync_read_metadata_page(uint8_t slot, uint8_t page)
{
    read_address_t address = ble_sync_metadata_address(slot, page);

    return spi_nand_page_read(address, 0U,
                              ble_sync_metadata_page_buffer,
                              sizeof(ble_sync_metadata_page_buffer));
}

static int ble_sync_find_free_metadata_page(uint8_t slot,
                                            uint8_t start_page,
                                            uint8_t *free_page)
{
    for (uint8_t page = start_page; page < NAND_PAGES_PER_BLOCK; page++)
    {
        if (ble_sync_read_metadata_page(slot, page) != SPI_NAND_RET_OK)
        {
            return -1;
        }
        if (ble_sync_buffer_is_erased(ble_sync_metadata_page_buffer,
                                      NAND_PAGE_SIZE_BYTES) != 0U)
        {
            *free_page = page;
            return 1;
        }
    }
    return 0;
}

static int ble_sync_program_metadata_record(uint8_t slot,
                                            uint8_t page,
                                            const BleSyncMetadataV1 *record)
{
    read_address_t address = ble_sync_metadata_address(slot, page);
    const BleSyncMetadataV1 *readback;

    if (spi_nand_page_program(address, 0U,
                              (const uint8_t *)record,
                              sizeof(*record)) != SPI_NAND_RET_OK)
    {
        ble_sync_metadata_write_errors++;
        return -1;
    }

    if (ble_sync_read_metadata_page(slot, page) != SPI_NAND_RET_OK)
    {
        ble_sync_metadata_write_errors++;
        return -1;
    }

    readback = (const BleSyncMetadataV1 *)ble_sync_metadata_page_buffer;
    if (!BleSync_MetadataIsValid(readback) ||
        (memcmp(readback, record, sizeof(*record)) != 0) ||
        (ble_sync_metadata_tail_is_erased(ble_sync_metadata_page_buffer) == 0U))
    {
        ble_sync_metadata_write_errors++;
        return -1;
    }

    ble_sync_metadata_records_written++;
    return 0;
}

static int ble_sync_append_metadata_record(const BleSyncMetadataV1 *record)
{
    BleSyncContext *context = &ble_sync_context;
    uint8_t page;
    uint8_t target_slot;
    int free_result;
    read_address_t erase_address;

    free_result = ble_sync_find_free_metadata_page(
            context->metadata_active_slot,
            context->metadata_next_page,
            &page);
    if (free_result < 0)
    {
        ble_sync_metadata_write_errors++;
        return -1;
    }

    if (free_result == 0)
    {
        target_slot = (uint8_t)(1U - context->metadata_active_slot);
        erase_address = ble_sync_metadata_address(target_slot, 0U);
        if (spi_nand_block_erase(erase_address) != SPI_NAND_RET_OK)
        {
            ble_sync_metadata_erase_errors++;
            return -1;
        }

        if (ble_sync_program_metadata_record(target_slot, 0U, record) != 0)
        {
            return -1;
        }

        /* The previous block remains intact until this copy is verified. */
        context->metadata_active_slot = target_slot;
        context->metadata_next_page = 1U;
    }
    else
    {
        if (ble_sync_program_metadata_record(context->metadata_active_slot,
                                             page, record) != 0)
        {
            return -1;
        }
        context->metadata_next_page = (uint8_t)(page + 1U);
    }

    context->metadata_sequence = record->metadata_sequence;
    context->persisted_ack_valid =
            ((record->flags & BLE_SYNC_METADATA_FLAG_ACK_VALID) != 0U) ? 1U : 0U;
    ble_sync_log_generation = record->log_generation;
    ble_sync_acked_through_persisted =
            record->acked_through_page_sequence;
    return 0;
}

static int ble_sync_persist_state(uint32_t log_generation,
                                  uint8_t ack_valid,
                                  uint32_t acked_through)
{
    BleSyncMetadataV1 record;
    uint32_t next_metadata_sequence = ble_sync_next_nonzero_sequence(
            ble_sync_context.metadata_sequence);

    BleSync_BuildMetadata(&record,
                          next_metadata_sequence,
                          log_generation,
                          ack_valid != 0U,
                          acked_through);
    if (ble_sync_append_metadata_record(&record) != 0)
    {
        ble_sync_last_error = BLE_SYNC_ERROR_METADATA;
        return -1;
    }

    ble_sync_context.acked_pages_since_persist = 0U;
    return 0;
}

static uint8_t ble_sync_ack_state_is_dirty(void)
{
    if (ble_sync_ack_valid != ble_sync_context.persisted_ack_valid)
    {
        return 1U;
    }

    return ((ble_sync_ack_valid != 0U) &&
            (ble_sync_acked_through_ram !=
             ble_sync_acked_through_persisted)) ? 1U : 0U;
}

static int ble_sync_recover_metadata(void)
{
    BleSyncContext *context = &ble_sync_context;
    BleSyncMetadataV1 latest;
    uint8_t latest_valid = 0U;
    uint8_t latest_slot = 0U;
    uint8_t latest_page = 0U;
    uint8_t free_page = 0U;
    int free_result;

    ble_sync_metadata_records_recovered = 0U;
    ble_sync_metadata_crc_errors = 0U;

    for (uint8_t slot = 0U; slot < BLE_SYNC_METADATA_RESERVED_BLOCKS; slot++)
    {
        for (uint8_t page = 0U; page < NAND_PAGES_PER_BLOCK; page++)
        {
            const BleSyncMetadataV1 *candidate;

            if (ble_sync_read_metadata_page(slot, page) != SPI_NAND_RET_OK)
            {
                return -1;
            }

            if (ble_sync_buffer_is_erased(ble_sync_metadata_page_buffer,
                                          NAND_PAGE_SIZE_BYTES) != 0U)
            {
                continue;
            }

            candidate = (const BleSyncMetadataV1 *)ble_sync_metadata_page_buffer;
            if (BleSync_MetadataIsValid(candidate) &&
                (ble_sync_metadata_tail_is_erased(
                        ble_sync_metadata_page_buffer) != 0U))
            {
                ble_sync_metadata_records_recovered++;
                if ((latest_valid == 0U) ||
                    BleSync_MetadataSequenceIsNewer(
                            candidate->metadata_sequence,
                            latest.metadata_sequence))
                {
                    latest = *candidate;
                    latest_valid = 1U;
                    latest_slot = slot;
                    latest_page = page;
                }
            }
            else if (candidate->magic == BLE_SYNC_METADATA_MAGIC)
            {
                ble_sync_metadata_crc_errors++;
            }
        }
    }

    if (latest_valid != 0U)
    {
        context->metadata_active_slot = latest_slot;
        context->metadata_sequence = latest.metadata_sequence;
        context->persisted_ack_valid =
                ((latest.flags & BLE_SYNC_METADATA_FLAG_ACK_VALID) != 0U) ? 1U : 0U;
        ble_sync_log_generation = latest.log_generation;
        ble_sync_ack_valid = context->persisted_ack_valid;
        ble_sync_acked_through_ram = latest.acked_through_page_sequence;
        ble_sync_acked_through_persisted = latest.acked_through_page_sequence;

        free_result = ble_sync_find_free_metadata_page(
                latest_slot, (uint8_t)(latest_page + 1U), &free_page);
        if (free_result < 0)
        {
            return -1;
        }
        context->metadata_next_page =
                (free_result > 0) ? free_page : NAND_PAGES_PER_BLOCK;
        return 0;
    }

    context->metadata_active_slot = 0U;
    context->metadata_sequence = 0U;
    context->persisted_ack_valid = 0U;
    ble_sync_log_generation = 1U;
    ble_sync_ack_valid = 0U;
    ble_sync_acked_through_ram = 0U;
    ble_sync_acked_through_persisted = 0U;

    free_result = ble_sync_find_free_metadata_page(0U, 0U, &free_page);
    if (free_result < 0)
    {
        return -1;
    }
    if (free_result == 0)
    {
        context->metadata_active_slot = 1U;
        free_result = ble_sync_find_free_metadata_page(1U, 0U, &free_page);
        if (free_result <= 0)
        {
            return -1;
        }
    }
    context->metadata_next_page = free_page;

    return ble_sync_persist_state(1U, 0U, 0U);
}

static void ble_sync_queue_error(uint16_t code, uint32_t detail)
{
    ble_sync_context.pending_error = 1U;
    ble_sync_context.pending_error_code = code;
    ble_sync_context.pending_error_detail = detail;
    ble_sync_last_error = code;
}

static int ble_sync_send_frame(uint8_t message_type,
                               const uint8_t *payload,
                               uint16_t payload_length)
{
    BleSyncContext *context = &ble_sync_context;
    size_t frame_length;
    uint16_t message_sequence = context->tx_message_sequence++;

    frame_length = BleSync_EncodeFrame(message_type,
                                       message_sequence,
                                       payload,
                                       payload_length,
                                       ble_sync_tx_frame,
                                       sizeof(ble_sync_tx_frame));
    if ((frame_length == 0U) ||
        (BLE_TransmitTransparent(ble_sync_tx_frame,
                                (uint16_t)frame_length,
                                BLE_SYNC_UART_TX_TIMEOUT_MS) != 0))
    {
        ble_sync_uart_errors++;
        ble_sync_last_error = BLE_SYNC_ERROR_UART;
        return -1;
    }

    ble_sync_frames_tx++;
    ble_sync_latest.last_tx_message_type = message_type;
    ble_sync_latest.last_tx_message_sequence = message_sequence;
    return 0;
}

static void ble_sync_handle_frame(const BleSyncFrame *frame)
{
    BleSyncContext *context = &ble_sync_context;

    ble_sync_frames_rx++;
    ble_sync_latest.last_rx_message_type = frame->message_type;
    ble_sync_latest.last_rx_message_sequence = frame->message_sequence;

    switch (frame->message_type)
    {
        case BLE_MSG_SYNC_START:
            if ((frame->payload_length == 0U) &&
                ((ble_sync_state == BLE_SYNC_PREPARE) ||
                 (ble_sync_state == BLE_SYNC_WAIT_START)))
            {
                context->sync_start_received = 1U;
            }
            else
            {
                ble_sync_queue_error(BLE_SYNC_ERROR_UNEXPECTED_MESSAGE,
                                     frame->message_type);
            }
            break;

        case BLE_MSG_ACK_THROUGH:
            if (frame->payload_length == BLE_SYNC_ACK_PAYLOAD_BYTES)
            {
                uint32_t generation = ble_sync_get_u32_le(&frame->payload[0]);
                uint32_t sequence = ble_sync_get_u32_le(&frame->payload[4]);

                ble_sync_latest.last_ack_generation =
                        generation;
                ble_sync_latest.last_ack_page_sequence =
                        sequence;

                if ((ble_sync_ack_valid != 0U) &&
                    (generation == ble_sync_log_generation) &&
                    (sequence == ble_sync_acked_through_ram))
                {
                    ble_sync_duplicate_acks++;
                }
                else if (ble_sync_state == BLE_SYNC_WAIT_ACK)
                {
                    context->pending_ack = 1U;
                    context->pending_ack_generation = generation;
                    context->pending_ack_sequence = sequence;
                }
                else
                {
                    ble_sync_invalid_acks++;
                    ble_sync_queue_error(BLE_SYNC_ERROR_INVALID_ACK, sequence);
                }
            }
            else
            {
                ble_sync_queue_error(BLE_SYNC_ERROR_FRAME_LENGTH,
                                     frame->payload_length);
            }
            break;

        case BLE_MSG_NACK_PAGE:
            if (frame->payload_length == BLE_SYNC_NACK_PAYLOAD_BYTES)
            {
                if (ble_sync_state == BLE_SYNC_WAIT_ACK)
                {
                    context->pending_nack = 1U;
                    context->pending_nack_generation =
                            ble_sync_get_u32_le(&frame->payload[0]);
                    context->pending_nack_sequence =
                            ble_sync_get_u32_le(&frame->payload[4]);
                    context->pending_nack_reason = frame->payload[8];
                }
                else
                {
                    ble_sync_queue_error(BLE_SYNC_ERROR_INVALID_NACK,
                                         frame->message_type);
                }
            }
            else
            {
                ble_sync_queue_error(BLE_SYNC_ERROR_FRAME_LENGTH,
                                     frame->payload_length);
            }
            break;

        case BLE_MSG_SYNC_ABORT:
            ble_sync_abort_requested = 1U;
            context->abort_reason = BLE_SYNC_ABORT_USER;
            break;

        default:
            ble_sync_queue_error(BLE_SYNC_ERROR_UNEXPECTED_MESSAGE,
                                 frame->message_type);
            break;
    }
}

static void ble_sync_pump_rx(uint32_t now_ms)
{
    BleSyncContext *context = &ble_sync_context;

    while (ble_sync_rx_tail != ble_sync_rx_head)
    {
        BleSyncFrame frame;
        BleSyncParseResult result;
        uint8_t byte = ble_sync_rx_ring[ble_sync_rx_tail];

        ble_sync_rx_tail = (uint16_t)((ble_sync_rx_tail + 1U) &
                                      (BLE_SYNC_RX_RING_BYTES - 1U));
        result = BleSync_ParseByte(&context->parser, byte, now_ms, &frame);

        if (result == BLE_SYNC_PARSE_FRAME)
        {
            ble_sync_handle_frame(&frame);
        }
        else if (result == BLE_SYNC_PARSE_CRC_ERROR)
        {
            ble_sync_frame_crc_errors++;
            ble_sync_queue_error(BLE_SYNC_ERROR_FRAME_CRC, 0U);
        }
        else if (result == BLE_SYNC_PARSE_LENGTH_ERROR)
        {
            ble_sync_frame_length_errors++;
            ble_sync_queue_error(BLE_SYNC_ERROR_FRAME_LENGTH, 0U);
        }
        else if (result == BLE_SYNC_PARSE_VERSION_ERROR)
        {
            ble_sync_protocol_version_errors++;
            ble_sync_queue_error(BLE_SYNC_ERROR_PROTOCOL_VERSION, 0U);
        }
    }

    if (BleSync_ParserHasTimedOut(&context->parser,
                                  now_ms,
                                  BLE_SYNC_FRAME_TIMEOUT_MS))
    {
        BleSync_ResetParser(&context->parser);
        ble_sync_frame_length_errors++;
        ble_sync_queue_error(BLE_SYNC_ERROR_FRAME_TIMEOUT, 0U);
    }
}

static int ble_sync_send_pending_error(void)
{
    uint8_t payload[BLE_SYNC_ERROR_PAYLOAD_BYTES] = {0};

    if (ble_sync_context.pending_error == 0U)
    {
        return 0;
    }

    ble_sync_put_u16_le(&payload[0], ble_sync_context.pending_error_code);
    ble_sync_put_u32_le(&payload[4], ble_sync_context.pending_error_detail);
    ble_sync_context.pending_error = 0U;
    return ble_sync_send_frame(BLE_MSG_ERROR, payload, sizeof(payload));
}

static void ble_sync_set_abort(BleSyncAbortReason reason)
{
    ble_sync_context.abort_reason = reason;
    ble_sync_last_abort_reason = reason;
    ble_sync_state = BLE_SYNC_ABORT;
}

static int ble_sync_prepare_snapshot_step(NandLogger *logger)
{
    BleSyncContext *context = &ble_sync_context;
    uint32_t processed = 0U;

    if (context->prepare_started == 0U)
    {
        if (NANDLogger_FlushAll(logger, HAL_GetTick()) != LOG_OK)
        {
            ble_sync_last_error = BLE_SYNC_ERROR_NAND_FLUSH;
            return -1;
        }

        context->prepare_started = 1U;
        context->snapshot_used_pages = logger->used_page_count;
        context->snapshot_cursor = 0U;
        context->snapshot_sequence_seen = 0U;
        context->snapshot_last_sequence = 0U;
        context->high_watermark_valid = 0U;
        context->oldest_available_sequence = 0U;
        context->newest_available_sequence = 0U;
        context->first_sequence_to_send = 0U;
        context->total_unsynced_pages = 0U;
        ble_sync_high_watermark = 0U;
        ble_sync_high_watermark_physical_page = UINT32_MAX;
        ble_sync_pages_planned = 0U;
    }

    while ((context->snapshot_cursor < context->snapshot_used_pages) &&
           (processed < BLE_SYNC_SCAN_PAGES_PER_PROCESS))
    {
        NandLoggerPageInfo info;

        if (NANDLogger_ReadPageInfo(logger,
                                    context->snapshot_cursor,
                                    &info) != LOG_OK)
        {
            ble_sync_last_error = BLE_SYNC_ERROR_NAND_READ;
            return -1;
        }
        context->snapshot_cursor++;
        processed++;

        if ((info.structurally_valid == 0U) ||
            ((context->snapshot_sequence_seen != 0U) &&
             (info.header.page_sequence <= context->snapshot_last_sequence)))
        {
            continue;
        }

        context->snapshot_last_sequence = info.header.page_sequence;
        if (context->snapshot_sequence_seen == 0U)
        {
            context->snapshot_sequence_seen = 1U;
            context->oldest_available_sequence = info.header.page_sequence;
        }
        context->newest_available_sequence = info.header.page_sequence;
        context->high_watermark_valid = 1U;
        ble_sync_high_watermark = info.header.page_sequence;
        ble_sync_high_watermark_physical_page = info.physical_page_index;

        if ((ble_sync_ack_valid == 0U) ||
            (info.header.page_sequence > ble_sync_acked_through_ram))
        {
            if (context->total_unsynced_pages == 0U)
            {
                context->first_sequence_to_send = info.header.page_sequence;
            }
            context->total_unsynced_pages++;
        }
    }

    if (context->snapshot_cursor < context->snapshot_used_pages)
    {
        return 0;
    }

    ble_sync_pages_planned = context->total_unsynced_pages;
    context->find_cursor = 0U;
    context->find_sequence_seen = 0U;
    context->find_last_sequence = 0U;
    context->start_deadline_ms = HAL_GetTick() + BLE_SYNC_START_TIMEOUT_MS;
    ble_sync_state = BLE_SYNC_WAIT_START;
    return 1;
}

static int ble_sync_send_status(void)
{
    BleSyncContext *context = &ble_sync_context;
    uint8_t payload[BLE_SYNC_STATUS_PAYLOAD_BYTES] = {0};
    uint32_t uid[3];

    uid[0] = HAL_GetUIDw0();
    uid[1] = HAL_GetUIDw1();
    uid[2] = HAL_GetUIDw2();
    memcpy(&payload[0], uid, sizeof(uid));
    ble_sync_put_u32_le(&payload[12], ble_sync_log_generation);
    payload[16] = ble_sync_ack_valid;
    ble_sync_put_u32_le(&payload[20],
                        (ble_sync_ack_valid != 0U) ?
                        ble_sync_acked_through_ram : 0U);
    ble_sync_put_u32_le(&payload[24],
                        context->high_watermark_valid ?
                        context->oldest_available_sequence : 0U);
    ble_sync_put_u32_le(&payload[28],
                        context->high_watermark_valid ?
                        context->newest_available_sequence : 0U);
    ble_sync_put_u32_le(&payload[32],
                        (context->total_unsynced_pages != 0U) ?
                        context->first_sequence_to_send : 0U);
    ble_sync_put_u32_le(&payload[36],
                        context->high_watermark_valid ?
                        ble_sync_high_watermark : 0U);
    ble_sync_put_u32_le(&payload[40], context->total_unsynced_pages);

    return ble_sync_send_frame(BLE_MSG_SYNC_STATUS,
                               payload, sizeof(payload));
}

static int ble_sync_find_next_page_step(NandLogger *logger)
{
    BleSyncContext *context = &ble_sync_context;
    uint32_t processed = 0U;

    while ((context->find_cursor < context->snapshot_used_pages) &&
           (processed < BLE_SYNC_SCAN_PAGES_PER_PROCESS))
    {
        NandLoggerPageInfo info;

        if (NANDLogger_ReadPageInfo(logger,
                                    context->find_cursor,
                                    &info) != LOG_OK)
        {
            ble_sync_last_error = BLE_SYNC_ERROR_NAND_READ;
            return -1;
        }
        context->find_cursor++;
        processed++;

        if ((info.structurally_valid == 0U) ||
            ((context->find_sequence_seen != 0U) &&
             (info.header.page_sequence <= context->find_last_sequence)))
        {
            continue;
        }

        context->find_sequence_seen = 1U;
        context->find_last_sequence = info.header.page_sequence;

        if (((ble_sync_ack_valid != 0U) &&
             (info.header.page_sequence <= ble_sync_acked_through_ram)) ||
            (info.header.page_sequence > ble_sync_high_watermark))
        {
            continue;
        }

        context->current_logical_page = info.logical_page_index;
        context->current_header = info.header;
        context->current_logical_bytes =
                (uint32_t)info.header.header_size +
                info.header.payload_bytes;
        if ((context->current_logical_bytes > NAND_PAGE_SIZE_BYTES) ||
            (NANDLogger_ReadPageBytes(logger,
                                      info.logical_page_index,
                                      0U,
                                      ble_sync_page_buffer,
                                      context->current_logical_bytes) != LOG_OK))
        {
            ble_sync_last_error = BLE_SYNC_ERROR_NAND_READ;
            return -1;
        }

        context->current_page_crc32 = BleSync_Crc32IsoHdlc(
                ble_sync_page_buffer, context->current_logical_bytes);
        context->current_data_offset = 0U;
        context->current_page_sent = 0U;
        ble_sync_current_page_sequence = info.header.page_sequence;
        ble_sync_current_physical_page = info.physical_page_index;
        ble_sync_current_retry_count = 0U;
        ble_sync_pages_started++;

        memcpy((void *)ble_sync_latest.current_page_magic,
               ble_sync_page_buffer, 4U);
        ble_sync_latest.current_page_header_size = info.header.header_size;
        ble_sync_latest.current_page_payload_bytes = info.header.payload_bytes;
        ble_sync_latest.current_page_logical_bytes =
                context->current_logical_bytes;
        ble_sync_latest.current_page_crc32 = context->current_page_crc32;
        return 1;
    }

    return (context->find_cursor >= context->snapshot_used_pages) ? 2 : 0;
}

static int ble_sync_send_page_begin(void)
{
    BleSyncContext *context = &ble_sync_context;
    uint8_t payload[BLE_SYNC_PAGE_BEGIN_PAYLOAD_BYTES] = {0};

    ble_sync_put_u32_le(&payload[0], ble_sync_log_generation);
    ble_sync_put_u32_le(&payload[4], ble_sync_current_page_sequence);
    ble_sync_put_u32_le(&payload[8], ble_sync_current_physical_page);
    ble_sync_put_u32_le(&payload[12], context->current_logical_bytes);
    ble_sync_put_u32_le(&payload[16], context->current_page_crc32);
    memcpy(&payload[20], ble_sync_page_buffer, 4U);
    payload[24] = context->current_header.version;
    payload[25] = context->current_header.header_size;
    ble_sync_put_u32_le(&payload[28], context->current_header.payload_bytes);

    return ble_sync_send_frame(BLE_MSG_PAGE_BEGIN,
                               payload, sizeof(payload));
}

static int ble_sync_send_page_data(void)
{
    BleSyncContext *context = &ble_sync_context;
    uint8_t payload[16U + BLE_SYNC_CHUNK_DATA_MAX] = {0};
    uint32_t remaining = context->current_logical_bytes -
                         context->current_data_offset;
    uint16_t chunk_length = (remaining > BLE_SYNC_CHUNK_DATA_MAX) ?
                            BLE_SYNC_CHUNK_DATA_MAX : (uint16_t)remaining;

    ble_sync_put_u32_le(&payload[0], ble_sync_log_generation);
    ble_sync_put_u32_le(&payload[4], ble_sync_current_page_sequence);
    ble_sync_put_u32_le(&payload[8], context->current_data_offset);
    ble_sync_put_u16_le(&payload[12], chunk_length);
    memcpy(&payload[16],
           &ble_sync_page_buffer[context->current_data_offset],
           chunk_length);

    if (ble_sync_send_frame(BLE_MSG_PAGE_DATA,
                            payload,
                            (uint16_t)(16U + chunk_length)) != 0)
    {
        return -1;
    }

    context->current_data_offset += chunk_length;
    return 0;
}

static int ble_sync_send_page_end(void)
{
    BleSyncContext *context = &ble_sync_context;
    uint8_t payload[BLE_SYNC_PAGE_END_PAYLOAD_BYTES] = {0};

    ble_sync_put_u32_le(&payload[0], ble_sync_log_generation);
    ble_sync_put_u32_le(&payload[4], ble_sync_current_page_sequence);
    ble_sync_put_u32_le(&payload[8], context->current_logical_bytes);
    ble_sync_put_u32_le(&payload[12], context->current_page_crc32);
    return ble_sync_send_frame(BLE_MSG_PAGE_END,
                               payload, sizeof(payload));
}

static void ble_sync_retry_current_page(BleSyncAbortReason exhausted_reason)
{
    if (ble_sync_current_retry_count >= BLE_SYNC_MAX_PAGE_RETRIES)
    {
        ble_sync_set_abort(exhausted_reason);
        return;
    }

    ble_sync_current_retry_count++;
    ble_sync_pages_retransmitted++;
    ble_sync_context.current_data_offset = 0U;
    ble_sync_context.current_page_sent = 0U;
    ble_sync_state = BLE_SYNC_SEND_PAGE_BEGIN;
}

static void ble_sync_process_ack(void)
{
    BleSyncContext *context = &ble_sync_context;
    uint32_t generation = context->pending_ack_generation;
    uint32_t sequence = context->pending_ack_sequence;

    context->pending_ack = 0U;

    if ((ble_sync_ack_valid != 0U) &&
        (generation == ble_sync_log_generation) &&
        (sequence == ble_sync_acked_through_ram))
    {
        ble_sync_duplicate_acks++;
        return;
    }

    if ((ble_sync_state != BLE_SYNC_WAIT_ACK) ||
        (generation != ble_sync_log_generation) ||
        (context->high_watermark_valid == 0U) ||
        (sequence > ble_sync_high_watermark) ||
        (context->current_page_sent == 0U) ||
        (sequence != ble_sync_current_page_sequence) ||
        ((ble_sync_ack_valid != 0U) &&
         (sequence < ble_sync_acked_through_ram)))
    {
        ble_sync_invalid_acks++;
        ble_sync_queue_error(BLE_SYNC_ERROR_INVALID_ACK, sequence);
        return;
    }

    ble_sync_ack_valid = 1U;
    ble_sync_acked_through_ram = sequence;
    context->pending_nack = 0U;
    context->acked_pages_since_persist++;
    ble_sync_pages_acked++;

    if (context->acked_pages_since_persist >=
        BLE_SYNC_ACK_PERSIST_INTERVAL_PAGES)
    {
        ble_sync_state = BLE_SYNC_PERSIST_ACK;
    }
    else
    {
        ble_sync_state = BLE_SYNC_FIND_NEXT_PAGE;
    }
}

static void ble_sync_process_nack(void)
{
    BleSyncContext *context = &ble_sync_context;
    uint8_t reason = context->pending_nack_reason;

    context->pending_nack = 0U;
    if ((ble_sync_state != BLE_SYNC_WAIT_ACK) ||
        (context->pending_nack_generation != ble_sync_log_generation) ||
        (context->pending_nack_sequence != ble_sync_current_page_sequence) ||
        (reason < BLE_SYNC_NACK_PAGE_CRC) ||
        (reason > BLE_SYNC_NACK_GENERIC))
    {
        ble_sync_queue_error(BLE_SYNC_ERROR_INVALID_NACK,
                             context->pending_nack_sequence);
        return;
    }

    ble_sync_nacks_received++;
    ble_sync_retry_current_page(BLE_SYNC_ABORT_MAXIMUM_RETRIES);
}

static int ble_sync_send_complete(void)
{
    uint8_t payload[BLE_SYNC_COMPLETE_PAYLOAD_BYTES] = {0};

    ble_sync_put_u32_le(&payload[0], ble_sync_log_generation);
    ble_sync_put_u32_le(&payload[4],
                        (ble_sync_ack_valid != 0U) ?
                        ble_sync_acked_through_ram : 0U);
    ble_sync_put_u32_le(&payload[8],
                        ble_sync_context.high_watermark_valid ?
                        ble_sync_high_watermark : 0U);
    return ble_sync_send_frame(BLE_MSG_SYNC_COMPLETE,
                               payload, sizeof(payload));
}

static void ble_sync_send_abort(void)
{
    uint8_t payload[BLE_SYNC_ABORT_PAYLOAD_BYTES] = {0};

    ble_sync_put_u32_le(&payload[0], ble_sync_log_generation);
    ble_sync_put_u32_le(&payload[4],
                        (ble_sync_ack_valid != 0U) ?
                        ble_sync_acked_through_ram : 0U);
    ble_sync_put_u16_le(&payload[8],
                        (uint16_t)ble_sync_context.abort_reason);
    (void)ble_sync_send_frame(BLE_MSG_SYNC_ABORT,
                              payload, sizeof(payload));
}

static void ble_sync_finish_session(uint8_t completed, uint32_t now_ms)
{
    ble_sync_session_duration_ms = now_ms - ble_sync_session_start_ms;
    if (completed != 0U)
    {
        ble_sync_sessions_completed++;
        ble_sync_last_abort_reason = BLE_SYNC_ABORT_NONE;
    }
    else
    {
        ble_sync_sessions_aborted++;
        ble_sync_last_abort_reason = ble_sync_context.abort_reason;
    }

    ble_sync_active = 0U;
    ble_sync_requested = 0U;
    ble_sync_abort_requested = 0U;
    ble_sync_state = BLE_SYNC_IDLE;
    ble_sync_context.pending_ack = 0U;
    ble_sync_context.pending_nack = 0U;
    ble_sync_context.sync_start_received = 0U;
}

int BleSync_Init(NandLogger *logger)
{
    BleSyncContext *context = &ble_sync_context;

    if (logger == NULL)
    {
        return -1;
    }

    memset(context, 0, sizeof(*context));
    memset((void *)&ble_sync_latest, 0, sizeof(ble_sync_latest));
    context->logger = logger;
    ble_sync_metadata_block_a = logger->sync_metadata_block_a;
    ble_sync_metadata_block_b = logger->sync_metadata_block_b;
    BleSync_ResetParser(&context->parser);

    if (ble_sync_recover_metadata() != 0)
    {
        ble_sync_last_error = BLE_SYNC_ERROR_METADATA;
        return -1;
    }
    context->metadata_ready = 1U;

#if (BLE_SYNC_ENABLE_BOOT_SELF_TESTS != 0U)
    ble_sync_protocol_self_test_failures = BleSync_RunProtocolSelfTests();
#else
    ble_sync_protocol_self_test_failures = 0U;
#endif

    ble_sync_rx_head = 0U;
    ble_sync_rx_tail = 0U;
    BLE_FlushTransparentReceive();
    if (BLE_StartReceiveByteIT((uint8_t *)&ble_sync_uart_rx_byte) != 0)
    {
        ble_sync_uart_errors++;
        ble_sync_last_error = BLE_SYNC_ERROR_UART;
        return -1;
    }

    ble_sync_state = BLE_SYNC_IDLE;
    return 0;
}

int BleSync_StartNewLogGeneration(void)
{
    uint32_t new_generation;

    if (ble_sync_context.metadata_ready == 0U)
    {
        return -1;
    }

    new_generation = ble_sync_next_nonzero_sequence(ble_sync_log_generation);
    if (ble_sync_persist_state(new_generation, 0U, 0U) != 0)
    {
        return -1;
    }

    ble_sync_ack_valid = 0U;
    ble_sync_acked_through_ram = 0U;
    ble_sync_context.persisted_ack_valid = 0U;
    return 0;
}

int BleSync_FactoryReset(NandLogger *logger)
{
    BleSyncContext *context = &ble_sync_context;
    read_address_t erase_address;
    uint32_t new_generation;
    uint32_t primask;

    if ((logger == NULL) || (ble_sync_active != 0U))
    {
        ble_sync_last_error = BLE_SYNC_ERROR_METADATA;
        return -1;
    }

    new_generation = ble_sync_next_nonzero_sequence(ble_sync_log_generation);
    context->logger = logger;
    ble_sync_metadata_block_a = logger->sync_metadata_block_a;
    ble_sync_metadata_block_b = logger->sync_metadata_block_b;

    ble_sync_metadata_records_recovered = 0U;
    ble_sync_metadata_records_written = 0U;
    ble_sync_metadata_crc_errors = 0U;
    ble_sync_metadata_write_errors = 0U;
    ble_sync_metadata_erase_errors = 0U;

    for (uint8_t slot = 0U; slot < BLE_SYNC_METADATA_RESERVED_BLOCKS; slot++)
    {
        erase_address = ble_sync_metadata_address(slot, 0U);
        App_UpdateFactoryEraseLed();
        if (spi_nand_block_erase(erase_address) != SPI_NAND_RET_OK)
        {
            ble_sync_metadata_erase_errors++;
            ble_sync_last_error = BLE_SYNC_ERROR_METADATA;
            return -1;
        }
    }

    memset(context, 0, sizeof(*context));
    memset((void *)&ble_sync_latest, 0, sizeof(ble_sync_latest));
    context->logger = logger;
    context->metadata_ready = 1U;
    context->metadata_active_slot = 0U;
    context->metadata_next_page = 0U;
    context->metadata_sequence = 0U;
    context->persisted_ack_valid = 0U;
    BleSync_ResetParser(&context->parser);

    ble_sync_log_generation = new_generation;
    ble_sync_ack_valid = 0U;
    ble_sync_acked_through_ram = 0U;
    ble_sync_acked_through_persisted = 0U;

    if (ble_sync_persist_state(new_generation, 0U, 0U) != 0)
    {
        ble_sync_last_error = BLE_SYNC_ERROR_METADATA;
        return -1;
    }

    ble_sync_requested = 0U;
    ble_sync_abort_requested = 0U;
    ble_sync_active = 0U;
    ble_sync_state = BLE_SYNC_IDLE;
    ble_sync_pages_planned = 0U;
    ble_sync_high_watermark = 0U;
    ble_sync_high_watermark_physical_page = UINT32_MAX;
    ble_sync_current_page_sequence = 0U;
    ble_sync_current_physical_page = UINT32_MAX;
    ble_sync_current_retry_count = 0U;
    ble_sync_last_error = BLE_SYNC_ERROR_NONE;
    ble_sync_last_abort_reason = BLE_SYNC_ABORT_NONE;
    ble_sync_session_start_ms = 0U;
    ble_sync_session_duration_ms = 0U;

    (void)HAL_UART_AbortReceive(&huart3);
    primask = __get_PRIMASK();
    __disable_irq();
    ble_sync_rx_head = 0U;
    ble_sync_rx_tail = 0U;
    ble_sync_uart_fault_pending = 0U;
    ble_sync_uart_rx_byte = 0U;
    BLE_FlushTransparentReceive();
    if ((primask & 1U) == 0U)
    {
        __enable_irq();
    }

    if (BLE_StartReceiveByteIT((uint8_t *)&ble_sync_uart_rx_byte) != 0)
    {
        ble_sync_uart_errors++;
        ble_sync_last_error = BLE_SYNC_ERROR_UART;
        return -1;
    }

    return 0;
}

int BleSync_StartSession(NandLogger *logger, uint32_t now_ms)
{
    BleSyncContext *context = &ble_sync_context;

    if ((logger == NULL) || (context->metadata_ready == 0U) ||
        (ble_sync_active != 0U))
    {
        return -1;
    }

    context->logger = logger;
    context->prepare_started = 0U;
    context->sync_start_received = 0U;
    context->pending_ack = 0U;
    context->pending_nack = 0U;
    context->pending_error = 0U;
    context->abort_reason = BLE_SYNC_ABORT_NONE;
    context->tx_message_sequence = 0U;
    ble_sync_last_error = BLE_SYNC_ERROR_NONE;
    ble_sync_last_abort_reason = BLE_SYNC_ABORT_NONE;
    ble_sync_current_page_sequence = 0U;
    ble_sync_current_physical_page = UINT32_MAX;
    ble_sync_current_retry_count = 0U;
    ble_sync_session_start_ms = now_ms;
    ble_sync_session_duration_ms = 0U;
    ble_sync_active = 1U;
    ble_sync_requested = 0U;
    ble_sync_abort_requested = 0U;
    ble_sync_uart_fault_pending = 0U;
    ble_sync_sessions_started++;
    ble_sync_state = BLE_SYNC_REQUESTED;
    return 0;
}

void BleSync_RequestUsbPreemption(void)
{
    if (ble_sync_active != 0U)
    {
        ble_sync_context.abort_reason = BLE_SYNC_ABORT_USB_PREEMPTION;
        ble_sync_abort_requested = 1U;
    }
}

void BleSync_Process(NandLogger *logger, uint32_t now_ms, uint8_t usb_active)
{
    BleSyncContext *context = &ble_sync_context;
    int result;

    ble_sync_pump_rx(now_ms);

    if (ble_sync_active == 0U)
    {
        return;
    }

    if (usb_active != 0U)
    {
        BleSync_RequestUsbPreemption();
    }

    if (ble_sync_uart_fault_pending != 0U)
    {
        ble_sync_uart_fault_pending = 0U;
        ble_sync_set_abort(BLE_SYNC_ABORT_UART_ERROR);
    }

    if (ble_sync_abort_requested != 0U)
    {
        if (context->abort_reason == BLE_SYNC_ABORT_NONE)
        {
            context->abort_reason = BLE_SYNC_ABORT_USER;
        }
        ble_sync_set_abort(context->abort_reason);
    }

    if ((context->pending_error != 0U) &&
        (ble_sync_state != BLE_SYNC_ABORT) &&
        (ble_sync_state != BLE_SYNC_ERROR))
    {
        if (ble_sync_send_pending_error() != 0)
        {
            ble_sync_set_abort(BLE_SYNC_ABORT_UART_ERROR);
        }
    }

    switch (ble_sync_state)
    {
        case BLE_SYNC_REQUESTED:
            ble_sync_state = BLE_SYNC_PREPARE;
            break;

        case BLE_SYNC_PREPARE:
            result = ble_sync_prepare_snapshot_step(logger);
            if (result < 0)
            {
                ble_sync_set_abort(BLE_SYNC_ABORT_NAND_ERROR);
            }
            break;

        case BLE_SYNC_WAIT_START:
            if (context->sync_start_received != 0U)
            {
                context->sync_start_received = 0U;
                ble_sync_state = BLE_SYNC_SEND_STATUS;
            }
            else if (ble_sync_deadline_reached(now_ms,
                                               context->start_deadline_ms) != 0U)
            {
                ble_sync_set_abort(BLE_SYNC_ABORT_START_TIMEOUT);
            }
            break;

        case BLE_SYNC_SEND_STATUS:
            if (ble_sync_send_status() != 0)
            {
                ble_sync_set_abort(BLE_SYNC_ABORT_UART_ERROR);
            }
            else if (context->total_unsynced_pages == 0U)
            {
                ble_sync_state = BLE_SYNC_COMPLETE;
            }
            else
            {
                ble_sync_state = BLE_SYNC_FIND_NEXT_PAGE;
            }
            break;

        case BLE_SYNC_FIND_NEXT_PAGE:
            result = ble_sync_find_next_page_step(logger);
            if (result < 0)
            {
                ble_sync_set_abort(BLE_SYNC_ABORT_NAND_ERROR);
            }
            else if (result == 1)
            {
                ble_sync_state = BLE_SYNC_SEND_PAGE_BEGIN;
            }
            else if (result == 2)
            {
                ble_sync_state = BLE_SYNC_COMPLETE;
            }
            break;

        case BLE_SYNC_SEND_PAGE_BEGIN:
            if (ble_sync_send_page_begin() != 0)
            {
                ble_sync_set_abort(BLE_SYNC_ABORT_UART_ERROR);
            }
            else
            {
                context->current_data_offset = 0U;
                ble_sync_state = BLE_SYNC_SEND_PAGE_DATA;
            }
            break;

        case BLE_SYNC_SEND_PAGE_DATA:
            if (ble_sync_send_page_data() != 0)
            {
                ble_sync_set_abort(BLE_SYNC_ABORT_UART_ERROR);
            }
            else if (context->current_data_offset >=
                     context->current_logical_bytes)
            {
                ble_sync_state = BLE_SYNC_SEND_PAGE_END;
            }
            break;

        case BLE_SYNC_SEND_PAGE_END:
            if (ble_sync_send_page_end() != 0)
            {
                ble_sync_set_abort(BLE_SYNC_ABORT_UART_ERROR);
            }
            else
            {
                context->current_page_sent = 1U;
                ble_sync_pages_sent++;
                context->ack_deadline_ms = now_ms + BLE_SYNC_ACK_TIMEOUT_MS;
                ble_sync_state = BLE_SYNC_WAIT_ACK;
            }
            break;

        case BLE_SYNC_WAIT_ACK:
            if (context->pending_ack != 0U)
            {
                ble_sync_process_ack();
            }
            if ((ble_sync_state == BLE_SYNC_WAIT_ACK) &&
                (context->pending_nack != 0U))
            {
                ble_sync_process_nack();
            }
            if ((ble_sync_state == BLE_SYNC_WAIT_ACK) &&
                (ble_sync_deadline_reached(now_ms,
                                           context->ack_deadline_ms) != 0U))
            {
                ble_sync_ack_timeouts++;
                ble_sync_retry_current_page(
                        BLE_SYNC_ABORT_MAXIMUM_RETRIES);
            }
            break;

        case BLE_SYNC_PERSIST_ACK:
            if (ble_sync_persist_state(ble_sync_log_generation,
                                       ble_sync_ack_valid,
                                       ble_sync_acked_through_ram) != 0)
            {
                ble_sync_set_abort(
                        BLE_SYNC_ABORT_METADATA_PERSISTENCE_ERROR);
            }
            else
            {
                ble_sync_state = BLE_SYNC_FIND_NEXT_PAGE;
            }
            break;

        case BLE_SYNC_COMPLETE:
            if ((ble_sync_ack_state_is_dirty() != 0U) &&
                (ble_sync_persist_state(ble_sync_log_generation,
                                        ble_sync_ack_valid,
                                        ble_sync_acked_through_ram) != 0))
            {
                ble_sync_set_abort(
                        BLE_SYNC_ABORT_METADATA_PERSISTENCE_ERROR);
                break;
            }
            if (ble_sync_send_complete() != 0)
            {
                ble_sync_set_abort(BLE_SYNC_ABORT_UART_ERROR);
                break;
            }
            ble_sync_finish_session(1U, now_ms);
            break;

        case BLE_SYNC_ABORT:
        case BLE_SYNC_ERROR:
            if ((ble_sync_ack_state_is_dirty() != 0U) &&
                (ble_sync_persist_state(ble_sync_log_generation,
                                        ble_sync_ack_valid,
                                        ble_sync_acked_through_ram) != 0))
            {
                context->abort_reason =
                        BLE_SYNC_ABORT_METADATA_PERSISTENCE_ERROR;
                ble_sync_last_abort_reason = context->abort_reason;
            }
            ble_sync_send_abort();
            ble_sync_finish_session(0U, now_ms);
            break;

        case BLE_SYNC_IDLE:
        default:
            break;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t next_head;

    if (huart != &huart3)
    {
        return;
    }

    next_head = (uint16_t)((ble_sync_rx_head + 1U) &
                           (BLE_SYNC_RX_RING_BYTES - 1U));
    if (next_head != ble_sync_rx_tail)
    {
        ble_sync_rx_ring[ble_sync_rx_head] = ble_sync_uart_rx_byte;
        ble_sync_rx_head = next_head;
    }
    else
    {
        ble_sync_uart_errors++;
        ble_sync_last_error = BLE_SYNC_ERROR_RX_OVERFLOW;
        ble_sync_uart_fault_pending = 1U;
    }

    if (BLE_StartReceiveByteIT((uint8_t *)&ble_sync_uart_rx_byte) != 0)
    {
        ble_sync_uart_errors++;
        ble_sync_last_error = BLE_SYNC_ERROR_UART;
        ble_sync_uart_fault_pending = 1U;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart3)
    {
        return;
    }

    ble_sync_uart_errors++;
    ble_sync_last_error = BLE_SYNC_ERROR_UART;
    ble_sync_uart_fault_pending = 1U;
    if (BLE_StartReceiveByteIT((uint8_t *)&ble_sync_uart_rx_byte) != 0)
    {
        ble_sync_uart_errors++;
    }
}

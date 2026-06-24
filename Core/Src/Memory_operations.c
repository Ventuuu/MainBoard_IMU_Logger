/*
 * Memory_operations.c
 *
 * High-level operations for managing SPI NAND logging.
 *
 * New architecture:
 * - One sequential NAND logger for all data types.
 * - Sensor pages are marked with LOG_MAGIC_SENSOR = 'SENS'.
 * - Audio pages are marked with LOG_MAGIC_AUDIO  = 'AUD0'.
 * - Raw light pages are marked with LOG_MAGIC_LIGHT_RAW = 'LRAW'.
 *
 * Page format:
 *
 *  Byte offset
 *  0..15      LogPageHeader
 *  16..4095   Payload
 *
 * Sensor page payload:
 *  - 102 records/page
 *  - 40 bytes/record
 *  - 4080 bytes total payload
 *  - legacy light bytes are reserved for compatibility
 *
 * Audio page payload:
 *  - int16_t PCM samples
 *  - AUDIO_BUFFER_SIZE = 1024 samples -> 2048 bytes
 *
 * Raw light page payload:
 *  - up to 145 28-byte little-endian LightRawSampleRecord records.
 */

#include "string.h"
#include "stdio.h"
#include "stdbool.h"
#include "main.h"
#include "SPI.h"
#include "SPI_NAND.h"
#include "Memory_operations.h"
#include "usbd_cdc_if.h"

#include "led_driver.h"


_Static_assert(sizeof(LogPageHeader) == LOG_HEADER_SIZE_BYTES,
               "LogPageHeader size must remain 16 bytes");

#define USB_TX_TIMEOUT_MS 5000U

uint8_t audio_NAND_packet[4096] = {0};
uint8_t audio_pagina_scritta = 0;
uint16_t audio_b = 0;
uint16_t audio_blocco_scritto = 0;

read_address_t audio_blocco;
column_address_t audio_colonna = 0;

uint16_t bad_blocks[2048] = {0};
uint8_t bad_blocks2[2048] = {0};

/* -------------------------------------------------------------------------- */
/*                              Legacy globals                                */
/* -------------------------------------------------------------------------- */

/*
 * These extern variables are part of the old implementation.
 * They are kept only for compatibility with existing code.
 * The new logger does not need them.
 */

NAND_info data;

/* IMU/light old variables */
extern uint8_t NAND_packet[4096];
extern uint16_t sample;
extern uint16_t blocco_scritto;
extern uint8_t pagina_scritta;
extern uint16_t b;

extern read_address_t blocco;
extern column_address_t colonna;

extern uint8_t bad_blocks2[2048];

/* Audio old variables */
extern uint8_t audio_NAND_packet[4096];

extern uint16_t audio_blocco_scritto;
extern uint8_t audio_pagina_scritta;
extern uint16_t audio_b;

extern read_address_t audio_blocco;
extern column_address_t audio_colonna;

/*
 * Old good-block list declared in main.c.
 * Despite the name "bad_blocks", your old code stores GOOD blocks here.
 */
extern uint16_t bad_blocks[2048];


/* -------------------------------------------------------------------------- */
/*                         Private logger helper data                         */
/* -------------------------------------------------------------------------- */

/*
 * Static page buffer for audio.
 * Avoid allocating 4096 bytes on the stack.
 */
static uint8_t logger_audio_page_buffer[NAND_PAGE_SIZE_BYTES];

/*
 * Static page buffer for download.
 * Avoid allocating 4096 bytes on the stack.
 */
static uint8_t logger_download_page_buffer[NAND_PAGE_SIZE_BYTES];

#ifndef NAND_VERIFY_LRAW_AFTER_WRITE
#define NAND_VERIFY_LRAW_AFTER_WRITE 1U
#endif

#define LOG_LIGHT_RAW_PARTIAL_TAIL_MIN_BYTES 8U

volatile uint32_t light_records_appended = 0U;
volatile uint32_t light_pages_written = 0U;
volatile uint32_t light_partial_pages_flushed = 0U;
volatile uint32_t light_full_pages_flushed = 0U;
volatile uint32_t light_nand_write_failures = 0U;
volatile uint32_t light_nand_verify_failures = 0U;
volatile uint32_t light_payload_consistency_failures = 0U;
volatile uint32_t light_records_serialized = 0U;
volatile uint32_t light_records_counter_incremented = 0U;
volatile uint32_t light_page_buffer_resets = 0U;
volatile uint32_t light_buffer_resets_while_nonempty = 0U;
volatile uint32_t light_non_ff_records_before_flush = 0U;
volatile uint32_t light_first_ff_record_before_flush = UINT32_MAX;
volatile uint32_t light_first_ff_record_after_readback = UINT32_MAX;
volatile uint32_t light_first_partial_record_before_flush = UINT32_MAX;
volatile uint32_t light_first_partial_byte_offset_before_flush = UINT32_MAX;
volatile uint32_t light_first_partial_record_after_readback = UINT32_MAX;
volatile uint32_t light_first_partial_byte_offset_after_readback = UINT32_MAX;
volatile uint32_t light_page_type_switches = 0U;
volatile uint32_t light_wrong_buffer_failures = 0U;
volatile uintptr_t light_serialization_buffer_address = 0U;
volatile uintptr_t light_programmed_buffer_address = 0U;
volatile uint32_t light_prewrite_check_count = 0U;
volatile uint32_t light_prewrite_declared_payload_bytes = 0U;
volatile uint32_t light_prewrite_declared_record_count = 0U;
volatile uint32_t light_prewrite_first_all_ff_record = UINT32_MAX;
volatile uint32_t light_prewrite_first_partial_record = UINT32_MAX;
volatile uint32_t light_prewrite_last_non_ff_offset = UINT32_MAX;
volatile uint32_t light_prewrite_current_record_count = 0U;
volatile uint32_t usb_tx_submit_count = 0U;
volatile uint32_t usb_tx_complete_count = 0U;
volatile uint32_t usb_tx_busy_retry_count = 0U;
volatile uint32_t usb_tx_fail_count = 0U;
volatile uint32_t usb_tx_timeout_count = 0U;
volatile uint16_t usb_tx_last_length = 0U;
volatile uint8_t usb_tx_last_status = 0U;

volatile uint32_t nand_erase_attempts = 0U;
volatile uint32_t nand_erase_failures = 0U;
volatile uint16_t nand_first_failed_erase_block = UINT16_MAX;
volatile int32_t nand_last_erase_status = SPI_NAND_RET_OK;

static void logger_note_light_payload_consistency_failure(NandLogger *logger);


/* -------------------------------------------------------------------------- */
/*                         Private logger helper functions                    */
/* -------------------------------------------------------------------------- */

static uint32_t logger_time_to_ms(Time_Struct timestamp)
{
    uint32_t total_ms = 0U;

    total_ms += ((uint32_t)timestamp.hh)  * 3600000UL;
    total_ms += ((uint32_t)timestamp.mm)  * 60000UL;
    total_ms += ((uint32_t)timestamp.ss)  * 1000UL;
    total_ms += ((uint32_t)timestamp.sss);

    return total_ms;
}


static void logger_prepare_header(uint8_t *page,
                                  uint32_t magic,
                                  uint16_t payload_bytes,
                                  uint32_t page_sequence,
                                  uint32_t timestamp_ms)
{
    LogPageHeader header;

    header.magic = magic;
    header.version = 1U;
    header.header_size = (uint8_t)sizeof(LogPageHeader);
    header.payload_bytes = payload_bytes;
    header.page_sequence = page_sequence;
    header.timestamp_ms = timestamp_ms;

    memcpy(page, &header, sizeof(LogPageHeader));
}

static void logger_put_u16_le(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void logger_put_u32_le(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8U) & 0xFFU);
    dst[2] = (uint8_t)((value >> 16U) & 0xFFU);
    dst[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static void logger_serialize_light_raw_record(uint8_t *dst,
                                              const LightRawSampleRecord *record)
{
    if ((dst == NULL) || (record == NULL))
    {
        return;
    }

    /* Explicit little-endian layout, independent from struct padding. */
    logger_put_u32_le(&dst[0], record->sample_elapsed_ms);
    logger_put_u32_le(&dst[4], record->sample_index);
    logger_put_u16_le(&dst[8], record->f1_counts);
    logger_put_u16_le(&dst[10], record->f2_counts);
    logger_put_u16_le(&dst[12], record->f3_counts);
    logger_put_u16_le(&dst[14], record->f4_counts);
    logger_put_u16_le(&dst[16], record->f5_counts);
    logger_put_u16_le(&dst[18], record->f6_counts);
    logger_put_u16_le(&dst[20], record->f7_counts);
    logger_put_u16_le(&dst[22], record->f8_counts);
    logger_put_u16_le(&dst[24], record->clear_counts);
    logger_put_u16_le(&dst[26], record->nir_counts);
}

void NANDLogger_SerializeLightRawRecordForTest(uint8_t *dst,
                                               const LightRawSampleRecord *record)
{
    logger_serialize_light_raw_record(dst, record);
}

static uint32_t logger_get_u32_le(const uint8_t *src)
{
    return ((uint32_t)src[0]) |
           ((uint32_t)src[1] << 8U) |
           ((uint32_t)src[2] << 16U) |
           ((uint32_t)src[3] << 24U);
}

static uint8_t logger_record_is_all_ff(const uint8_t *record,
                                       uint32_t length)
{
    if (record == NULL)
    {
        return 1U;
    }

    for (uint32_t i = 0U; i < length; i++)
    {
        if (record[i] != 0xFFU)
        {
            return 0U;
        }
    }

    return 1U;
}

static uint32_t logger_record_first_erased_tail_offset(const uint8_t *record,
                                                       uint32_t length)
{
    if (record == NULL)
    {
        return 0U;
    }

    for (uint32_t i = 1U; i < length; i++)
    {
        uint32_t tail_len = length - i;
        uint8_t prefix_has_data = 0U;
        uint8_t tail_is_ff = 1U;

        if (tail_len < LOG_LIGHT_RAW_PARTIAL_TAIL_MIN_BYTES)
        {
            continue;
        }

        for (uint32_t p = 0U; p < i; p++)
        {
            if (record[p] != 0xFFU)
            {
                prefix_has_data = 1U;
                break;
            }
        }

        if (prefix_has_data == 0U)
        {
            continue;
        }

        for (uint32_t t = i; t < length; t++)
        {
            if (record[t] != 0xFFU)
            {
                tail_is_ff = 0U;
                break;
            }
        }

        if (tail_is_ff != 0U)
        {
            return i;
        }
    }

    return UINT32_MAX;
}

static uint8_t logger_light_raw_record_payload_is_valid(const uint8_t *record,
                                                        uint32_t *partial_byte_offset)
{
    if (record == NULL)
    {
        if (partial_byte_offset != NULL)
        {
            *partial_byte_offset = 0U;
        }
        return 0U;
    }

    if (logger_record_is_all_ff(record, LOG_LIGHT_RAW_RECORD_BYTES) != 0U)
    {
        if (partial_byte_offset != NULL)
        {
            *partial_byte_offset = 0U;
        }
        return 0U;
    }

    if ((logger_get_u32_le(&record[0]) == UINT32_MAX) ||
        (logger_get_u32_le(&record[4]) == UINT32_MAX))
    {
        if (partial_byte_offset != NULL)
        {
            *partial_byte_offset = 0U;
        }
        return 0U;
    }

    if (partial_byte_offset != NULL)
    {
        *partial_byte_offset =
            logger_record_first_erased_tail_offset(record, LOG_LIGHT_RAW_RECORD_BYTES);

        if (*partial_byte_offset != UINT32_MAX)
        {
            return 0U;
        }
    }

    return 1U;
}

static uint32_t logger_count_valid_light_raw_records(const uint8_t *page,
                                                     uint16_t record_count,
                                                     uint32_t *first_bad_record,
                                                     uint32_t *first_partial_byte_offset)
{
    uint32_t valid_records = 0U;

    if (first_bad_record != NULL)
    {
        *first_bad_record = UINT32_MAX;
    }

    if (first_partial_byte_offset != NULL)
    {
        *first_partial_byte_offset = UINT32_MAX;
    }

    if (page == NULL)
    {
        if (first_bad_record != NULL)
        {
            *first_bad_record = 0U;
        }
        return 0U;
    }

    for (uint16_t i = 0U; i < record_count; i++)
    {
        uint32_t offset = LOG_HEADER_SIZE_BYTES +
                          ((uint32_t)i * LOG_LIGHT_RAW_RECORD_BYTES);

        if ((offset + LOG_LIGHT_RAW_RECORD_BYTES) > NAND_PAGE_SIZE_BYTES)
        {
            if (first_bad_record != NULL)
            {
                *first_bad_record = i;
            }
            return valid_records;
        }

        uint32_t partial_byte_offset = UINT32_MAX;

        if (logger_light_raw_record_payload_is_valid(&page[offset],
                                                     &partial_byte_offset) == 0U)
        {
            if (first_bad_record != NULL)
            {
                *first_bad_record = i;
            }

            if (first_partial_byte_offset != NULL)
            {
                *first_partial_byte_offset = partial_byte_offset;
            }

            return valid_records;
        }

        valid_records++;
    }

    return valid_records;
}

static void logger_capture_light_prewrite_diagnostics(const NandLogger *logger)
{
    const uint8_t *page;
    LogPageHeader header;
    uint32_t declared_record_count;
    uint32_t records_to_scan;

    light_prewrite_check_count++;
    light_prewrite_declared_payload_bytes = 0U;
    light_prewrite_declared_record_count = 0U;
    light_prewrite_first_all_ff_record = UINT32_MAX;
    light_prewrite_first_partial_record = UINT32_MAX;
    light_prewrite_last_non_ff_offset = UINT32_MAX;
    light_prewrite_current_record_count = 0U;

    if (logger == NULL)
    {
        return;
    }

    page = logger->light_raw_page_buffer;
    light_prewrite_current_record_count = logger->light_raw_records_in_page;

    memcpy(&header, page, sizeof(header));
    light_prewrite_declared_payload_bytes = header.payload_bytes;

    declared_record_count =
        ((uint32_t)header.payload_bytes) / LOG_LIGHT_RAW_RECORD_BYTES;
    light_prewrite_declared_record_count = declared_record_count;

    records_to_scan = declared_record_count;
    if (records_to_scan > LOG_LIGHT_RAW_RECORDS_PER_PAGE)
    {
        records_to_scan = LOG_LIGHT_RAW_RECORDS_PER_PAGE;
    }

    for (uint32_t offset = 0U; offset < NAND_PAGE_SIZE_BYTES; offset++)
    {
        if (page[offset] != 0xFFU)
        {
            light_prewrite_last_non_ff_offset = offset;
        }
    }

    for (uint32_t record_index = 0U; record_index < records_to_scan; record_index++)
    {
        uint32_t record_offset = LOG_HEADER_SIZE_BYTES +
                                 (record_index * LOG_LIGHT_RAW_RECORD_BYTES);
        uint8_t has_data = 0U;
        uint8_t has_ff = 0U;

        if ((record_offset + LOG_LIGHT_RAW_RECORD_BYTES) > NAND_PAGE_SIZE_BYTES)
        {
            break;
        }

        for (uint32_t byte_index = 0U;
             byte_index < LOG_LIGHT_RAW_RECORD_BYTES;
             byte_index++)
        {
            if (page[record_offset + byte_index] == 0xFFU)
            {
                has_ff = 1U;
            }
            else
            {
                has_data = 1U;
            }
        }

        if ((has_data == 0U) &&
            (has_ff != 0U) &&
            (light_prewrite_first_all_ff_record == UINT32_MAX))
        {
            light_prewrite_first_all_ff_record = record_index;
        }

        if ((has_data != 0U) &&
            (has_ff != 0U) &&
            (light_prewrite_first_partial_record == UINT32_MAX))
        {
            light_prewrite_first_partial_record = record_index;
        }
    }
}

static void logger_reset_light_raw_page_buffer(NandLogger *logger,
                                               uint8_t after_successful_flush)
{
    light_page_buffer_resets++;

    if ((logger != NULL) &&
        (after_successful_flush == 0U) &&
        ((logger->light_raw_records_in_page != 0U) ||
         (logger->light_raw_payload_bytes != 0U)))
    {
        light_buffer_resets_while_nonempty++;
        logger_note_light_payload_consistency_failure(logger);
    }

    if (logger != NULL)
    {
        memset(logger->light_raw_page_buffer, 0xFF, NAND_PAGE_SIZE_BYTES);
    }
}


static LogStatus logger_advance_page(NandLogger *logger)
{
    if (logger == NULL)
    {
        return LOG_ERR_BAD_ARGUMENT;
    }

    logger->current_page_in_block++;

    if (logger->current_page_in_block >= NAND_PAGES_PER_BLOCK)
    {
        logger->current_page_in_block = 0U;
        logger->current_good_block_index++;
    }

    return LOG_OK;
}

static LogStatus logger_logical_page_to_address(const NandLogger *logger,
                                                uint32_t logical_page,
                                                read_address_t *addr)
{
    uint16_t good_block_index;

    if ((logger == NULL) || (addr == NULL))
    {
        return LOG_ERR_BAD_ARGUMENT;
    }

    good_block_index = (uint16_t)(logical_page / NAND_PAGES_PER_BLOCK);

    if (good_block_index >= logger->good_block_count)
    {
        return LOG_ERR_FULL;
    }

    addr->block = logger->good_blocks[good_block_index];
    addr->page = (uint8_t)(logical_page % NAND_PAGES_PER_BLOCK);
    addr->dummy = 0U;

    return LOG_OK;
}

static LogStatus logger_write_current_page(NandLogger *logger,
                                           const uint8_t *page)
{
    read_address_t addr;
    column_address_t column = 0;
    int nand_ret;

    if ((logger == NULL) || (page == NULL))
    {
        return LOG_ERR_BAD_ARGUMENT;
    }

    if (logger->good_block_count == 0U)
    {
        return LOG_ERR_NO_GOOD_BLOCKS;
    }

    if (logger->current_good_block_index >= logger->good_block_count)
    {
        return LOG_ERR_FULL;
    }

    addr.block = logger->good_blocks[logger->current_good_block_index];
    addr.page = logger->current_page_in_block;
    addr.dummy = 0U;

    /*
     * Low-level NAND page program.
     * This function is assumed to be provided by SPI_NAND.c.
     */
    nand_ret = spi_nand_page_program(addr,
                                     column,
                                     page,
                                     NAND_PAGE_SIZE_BYTES);

    if (nand_ret != SPI_NAND_RET_OK)
    {
        return LOG_ERR_NAND;
    }

    logger->page_sequence++;

    return logger_advance_page(logger);
}

static bool logger_light_raw_payload_is_consistent(const NandLogger *logger)
{
    if (logger == NULL)
    {
        return false;
    }

    if ((logger->light_raw_payload_bytes % LOG_LIGHT_RAW_RECORD_BYTES) != 0U)
    {
        return false;
    }

    if (logger->light_raw_payload_bytes > LOG_LIGHT_RAW_MAX_PAYLOAD_BYTES)
    {
        return false;
    }

    if (logger->light_raw_records_in_page > LOG_LIGHT_RAW_RECORDS_PER_PAGE)
    {
        return false;
    }

    return (logger->light_raw_payload_bytes ==
            (logger->light_raw_records_in_page * LOG_LIGHT_RAW_RECORD_BYTES));
}

static void logger_note_light_payload_consistency_failure(NandLogger *logger)
{
    light_payload_consistency_failures++;

    if (logger != NULL)
    {
        logger->light_payload_consistency_failures++;
    }
}

#if (NAND_VERIFY_LRAW_AFTER_WRITE != 0U)
static LogStatus logger_verify_light_raw_page(const NandLogger *logger,
                                              uint32_t page_sequence,
                                              const uint8_t *expected_page,
                                              uint16_t expected_payload_bytes)
{
    read_address_t addr;
    column_address_t column = 0U;
    LogPageHeader header;
    uint16_t record_count;
    uint32_t valid_records;
    uint32_t first_bad_record;
    uint32_t first_partial_byte_offset;
    int nand_ret;

    if ((logger == NULL) || (expected_page == NULL))
    {
        return LOG_ERR_BAD_ARGUMENT;
    }

    if (logger_logical_page_to_address(logger, page_sequence, &addr) != LOG_OK)
    {
        return LOG_ERR_FULL;
    }

    memset(logger_download_page_buffer, 0xFF, NAND_PAGE_SIZE_BYTES);
    nand_array_verify_count++;
    nand_array_verify_first_mismatch_offset = UINT32_MAX;

    nand_ret = spi_nand_page_read(addr,
                                  column,
                                  logger_download_page_buffer,
                                  NAND_PAGE_SIZE_BYTES);

    if (nand_ret != SPI_NAND_RET_OK)
    {
        nand_array_verify_first_mismatch_offset = 0U;
        return LOG_ERR_NAND;
    }

    for (uint32_t i = 0U; i < NAND_PAGE_SIZE_BYTES; i++)
    {
        if (logger_download_page_buffer[i] != expected_page[i])
        {
            nand_array_verify_first_mismatch_offset = i;
            return LOG_ERR_NAND;
        }
    }

    memcpy(&header, logger_download_page_buffer, sizeof(header));

    if ((header.magic != LOG_MAGIC_LIGHT_RAW) ||
        (header.version != 1U) ||
        (header.header_size != LOG_HEADER_SIZE_BYTES) ||
        (header.payload_bytes != expected_payload_bytes) ||
        (header.page_sequence != page_sequence) ||
        ((header.payload_bytes % LOG_LIGHT_RAW_RECORD_BYTES) != 0U) ||
        (header.payload_bytes > LOG_LIGHT_RAW_MAX_PAYLOAD_BYTES))
    {
        return LOG_ERR_NAND;
    }

    record_count = (uint16_t)(header.payload_bytes / LOG_LIGHT_RAW_RECORD_BYTES);
    valid_records = logger_count_valid_light_raw_records(logger_download_page_buffer,
                                                         record_count,
                                                         &first_bad_record,
                                                         &first_partial_byte_offset);
    light_first_ff_record_after_readback = first_bad_record;
    light_first_partial_record_after_readback = first_bad_record;
    light_first_partial_byte_offset_after_readback = first_partial_byte_offset;

    if (valid_records != record_count)
    {
        return LOG_ERR_NAND;
    }

    return LOG_OK;
}
#endif


static LogStatus logger_flush_sensor_page(NandLogger *logger,
                                          uint32_t timestamp_ms)
{
    uint16_t payload_bytes;
    LogStatus status;

    if (logger == NULL)
    {
        return LOG_ERR_BAD_ARGUMENT;
    }

    /*
     * Nothing to flush.
     */
    if (logger->sensor_records_in_page == 0U)
    {
        return LOG_OK;
    }

    payload_bytes = logger->sensor_records_in_page * LOG_SENSOR_RECORD_BYTES;

    logger_prepare_header(logger->sensor_page_buffer,
                          LOG_MAGIC_SENSOR,
                          payload_bytes,
                          logger->page_sequence,
                          timestamp_ms);

    status = logger_write_current_page(logger,
                                       logger->sensor_page_buffer);

    logger->sensor_records_in_page = 0U;
    memset(logger->sensor_page_buffer, 0xFF, NAND_PAGE_SIZE_BYTES);

    return status;
}

static LogStatus logger_flush_light_raw_page(NandLogger *logger,
                                             uint32_t timestamp_ms)
{
    uint16_t payload_bytes;
    uint32_t page_sequence;
    uint32_t valid_records;
    uint32_t first_bad_record;
    uint32_t first_partial_byte_offset;
    LogStatus status;

    if (logger == NULL)
    {
        return LOG_ERR_BAD_ARGUMENT;
    }

    if (logger->light_raw_records_in_page == 0U)
    {
        if (logger->light_raw_payload_bytes != 0U)
        {
            logger_note_light_payload_consistency_failure(logger);
            logger->light_raw_payload_bytes = 0U;
        }
        return LOG_OK;
    }

    if (!logger_light_raw_payload_is_consistent(logger))
    {
        logger_note_light_payload_consistency_failure(logger);
        return LOG_ERR_BAD_ARGUMENT;
    }

    valid_records = logger_count_valid_light_raw_records(logger->light_raw_page_buffer,
                                                         logger->light_raw_records_in_page,
                                                         &first_bad_record,
                                                         &first_partial_byte_offset);
    light_non_ff_records_before_flush = valid_records;
    light_first_ff_record_before_flush = first_bad_record;
    light_first_partial_record_before_flush = first_bad_record;
    light_first_partial_byte_offset_before_flush = first_partial_byte_offset;

    if (valid_records != logger->light_raw_records_in_page)
    {
        logger_note_light_payload_consistency_failure(logger);
        return LOG_ERR_BAD_ARGUMENT;
    }

    payload_bytes = logger->light_raw_payload_bytes;
    page_sequence = logger->page_sequence;

    logger_prepare_header(logger->light_raw_page_buffer,
                          LOG_MAGIC_LIGHT_RAW,
                          payload_bytes,
                          logger->page_sequence,
                          timestamp_ms);

    logger_capture_light_prewrite_diagnostics(logger);

    light_programmed_buffer_address = (uintptr_t)logger->light_raw_page_buffer;

    if (light_programmed_buffer_address == 0U)
    {
        light_wrong_buffer_failures++;
        logger_note_light_payload_consistency_failure(logger);
        return LOG_ERR_BAD_ARGUMENT;
    }

    status = logger_write_current_page(logger,
                                       logger->light_raw_page_buffer);

    if (status == LOG_OK)
    {
#if (NAND_VERIFY_LRAW_AFTER_WRITE != 0U)
        if (logger_verify_light_raw_page(logger,
                                         page_sequence,
                                         logger->light_raw_page_buffer,
                                         payload_bytes) != LOG_OK)
        {
            logger->light_nand_verify_failures++;
            light_nand_verify_failures++;
            return LOG_ERR_NAND;
        }
#endif

        logger->light_pages_written++;
        light_pages_written++;

        if (payload_bytes == LOG_LIGHT_RAW_MAX_PAYLOAD_BYTES)
        {
            logger->light_full_pages_flushed++;
            light_full_pages_flushed++;
        }
        else
        {
            logger->light_partial_pages_flushed++;
            light_partial_pages_flushed++;
        }

        logger->light_raw_records_in_page = 0U;
        logger->light_raw_payload_bytes = 0U;
        logger_reset_light_raw_page_buffer(logger, 1U);
    }
    else
    {
        logger->light_nand_write_failures++;
        light_nand_write_failures++;
    }

    return status;
}


/* -------------------------------------------------------------------------- */
/*                       New public logger functions                          */
/* -------------------------------------------------------------------------- */

LogStatus NANDLogger_Init(NandLogger *logger)
{
    read_address_t addr;
    bool is_bad_mark = true;

    if (logger == NULL)
    {
        return LOG_ERR_BAD_ARGUMENT;
    }

    memset(logger, 0, sizeof(NandLogger));

    addr.page = 0U;
    addr.dummy = 0U;

    for (uint16_t block = 0U; block < NAND_TOTAL_BLOCKS; block++)
    {
        addr.block = block;
        is_bad_mark = true;

        spi_nand_block_is_bad(addr, &is_bad_mark);

        if (!is_bad_mark)
        {
            logger->good_blocks[logger->good_block_count] = block;
            logger->good_block_count++;
        }
    }

    if (logger->good_block_count == 0U)
    {
        return LOG_ERR_NO_GOOD_BLOCKS;
    }

    logger->current_good_block_index = 0U;
    logger->current_page_in_block = 0U;
    logger->page_sequence = 0U;
    logger->sensor_records_in_page = 0U;
    logger->light_raw_records_in_page = 0U;
    logger->light_raw_payload_bytes = 0U;

    memset(logger->sensor_page_buffer, 0xFF, NAND_PAGE_SIZE_BYTES);
    logger_reset_light_raw_page_buffer(logger, 0U);

    return LOG_OK;
}


LogStatus NANDLogger_EraseAllGoodBlocks(NandLogger *logger)
{
    read_address_t addr;
    int erase_ret;

    if (logger == NULL)
    {
        return LOG_ERR_BAD_ARGUMENT;
    }

    if (logger->good_block_count == 0U)
    {
        return LOG_ERR_NO_GOOD_BLOCKS;
    }

    nand_erase_attempts = 0U;
    nand_erase_failures = 0U;
    nand_first_failed_erase_block = UINT16_MAX;
    nand_last_erase_status = SPI_NAND_RET_OK;

    addr.page = 0U;
    addr.dummy = 0U;

    for (uint16_t i = 0U; i < logger->good_block_count; i++)
    {
        addr.block = logger->good_blocks[i];

        nand_erase_attempts++;

        erase_ret = spi_nand_block_erase(addr);
        nand_last_erase_status = erase_ret;

        if (erase_ret != SPI_NAND_RET_OK)
        {
            nand_erase_failures++;

            if (nand_first_failed_erase_block == UINT16_MAX)
            {
                nand_first_failed_erase_block = addr.block;
            }

            return LOG_ERR_NAND;
        }
    }

    logger->current_good_block_index = 0U;
    logger->current_page_in_block = 0U;
    logger->page_sequence = 0U;
    logger->sensor_records_in_page = 0U;
    logger->light_raw_records_in_page = 0U;
    logger->light_raw_payload_bytes = 0U;
    logger->light_pages_written = 0U;
    logger->light_partial_pages_flushed = 0U;
    logger->light_full_pages_flushed = 0U;
    logger->light_nand_write_failures = 0U;
    logger->light_nand_verify_failures = 0U;
    logger->light_payload_consistency_failures = 0U;
    light_records_appended = 0U;
    light_pages_written = 0U;
    light_partial_pages_flushed = 0U;
    light_full_pages_flushed = 0U;
    light_nand_write_failures = 0U;
    light_nand_verify_failures = 0U;
    light_payload_consistency_failures = 0U;
    light_records_serialized = 0U;
    light_records_counter_incremented = 0U;
    light_page_buffer_resets = 0U;
    light_buffer_resets_while_nonempty = 0U;
    light_non_ff_records_before_flush = 0U;
    light_first_ff_record_before_flush = UINT32_MAX;
    light_first_ff_record_after_readback = UINT32_MAX;
    light_first_partial_record_before_flush = UINT32_MAX;
    light_first_partial_byte_offset_before_flush = UINT32_MAX;
    light_first_partial_record_after_readback = UINT32_MAX;
    light_first_partial_byte_offset_after_readback = UINT32_MAX;
    light_page_type_switches = 0U;
    light_wrong_buffer_failures = 0U;
    light_serialization_buffer_address = 0U;
    light_programmed_buffer_address = 0U;

    memset(logger->sensor_page_buffer, 0xFF, NAND_PAGE_SIZE_BYTES);
    logger_reset_light_raw_page_buffer(logger, 0U);

    return LOG_OK;
}


LogStatus NANDLogger_AppendSensorRecord(NandLogger *logger,
                                        Time_Struct timestamp,
                                        const uint8_t *accelerometer,
                                        const uint8_t *gyroscope,
                                        const uint8_t *light_raw)
{
    uint32_t timestamp_ms;

    if ((logger == NULL) ||
        (accelerometer == NULL) ||
        (gyroscope == NULL) ||
        (light_raw == NULL))
    {
        return LOG_ERR_BAD_ARGUMENT;
    }

    if (logger->current_good_block_index >= logger->good_block_count)
    {
        return LOG_ERR_FULL;
    }

    /*
     * New page: initialize it to erased-like state.
     */
    if (logger->sensor_records_in_page == 0U)
    {
        memset(logger->sensor_page_buffer, 0xFF, NAND_PAGE_SIZE_BYTES);
    }

    /*
     * Write one 40-byte sensor record after the 16-byte page header.
     *
     * write_packet() expects its buffer to start at offset 0 for sample 0.
     * Therefore we pass:
     *
     * &logger->sensor_page_buffer[LOG_HEADER_SIZE_BYTES]
     *
     * so that sample 0 starts at physical page offset 16.
     */
    write_packet(logger->sensor_records_in_page,
                 timestamp,
                 (uint8_t *)accelerometer,
                 (uint8_t *)gyroscope,
                 (uint8_t *)light_raw,
                 &logger->sensor_page_buffer[LOG_HEADER_SIZE_BYTES]);

    logger->sensor_records_in_page++;

    /*
     * When the sensor page is full, add the SENS header and write it to NAND.
     */
    if (logger->sensor_records_in_page >= LOG_SENSOR_RECORDS_PER_PAGE)
    {
        if (logger->light_raw_records_in_page != 0U)
        {
            light_page_type_switches++;
        }

        timestamp_ms = logger_time_to_ms(timestamp);
        return logger_flush_sensor_page(logger, timestamp_ms);
    }

    return LOG_OK;
}


LogStatus NANDLogger_AppendAudioBuffer(NandLogger *logger,
                                       const int16_t *audio_buffer,
                                       uint32_t audio_samples,
                                       uint32_t timestamp_ms)
{
    
    LED_On(LED_RED);
    
    uint32_t audio_bytes;
    
    if ((logger == NULL) || (audio_buffer == NULL))
    {
        return LOG_ERR_BAD_ARGUMENT;
    }
    
    if (logger->current_good_block_index >= logger->good_block_count)
    {
        return LOG_ERR_FULL;
    }
    
    audio_bytes = audio_samples * sizeof(int16_t);
    
    if (audio_bytes > (NAND_PAGE_SIZE_BYTES - LOG_HEADER_SIZE_BYTES))
    {
        return LOG_ERR_BAD_ARGUMENT;
    }

    if (logger->light_raw_records_in_page != 0U)
    {
        light_page_type_switches++;
    }
    
    /*
    * Audio is written as a complete page:
    *
    * [0..15]   LogPageHeader with LOG_MAGIC_AUDIO
    * [16..]    PCM int16_t samples
    */
    memset(logger_audio_page_buffer, 0xFF, sizeof(logger_audio_page_buffer));
    
    logger_prepare_header(logger_audio_page_buffer,
        LOG_MAGIC_AUDIO,
        (uint16_t)audio_bytes,
        logger->page_sequence,
        timestamp_ms);
        
        memcpy(&logger_audio_page_buffer[LOG_HEADER_SIZE_BYTES],
            (const uint8_t *)audio_buffer,
            audio_bytes);
            
            return logger_write_current_page(logger,
                logger_audio_page_buffer);   
            }

LogStatus NANDLogger_AppendLightRawRecord(NandLogger *logger,
                                          const LightRawSampleRecord *record,
                                          uint32_t timestamp_ms)
{
    uint32_t payload_offset;
    LogStatus status;

    if ((logger == NULL) || (record == NULL))
    {
        return LOG_ERR_BAD_ARGUMENT;
    }

    if (logger->current_good_block_index >= logger->good_block_count)
    {
        return LOG_ERR_FULL;
    }

    if (!logger_light_raw_payload_is_consistent(logger))
    {
        logger_note_light_payload_consistency_failure(logger);
        return LOG_ERR_BAD_ARGUMENT;
    }

    if ((logger->light_raw_payload_bytes + LOG_LIGHT_RAW_RECORD_BYTES) >
        LOG_LIGHT_RAW_MAX_PAYLOAD_BYTES)
    {
        status = logger_flush_light_raw_page(logger, timestamp_ms);
        if (status != LOG_OK)
        {
            return status;
        }
    }

    if (logger->light_raw_records_in_page == 0U)
    {
        logger_reset_light_raw_page_buffer(logger, 0U);
    }

    payload_offset = LOG_HEADER_SIZE_BYTES + logger->light_raw_payload_bytes;

    if ((payload_offset + LOG_LIGHT_RAW_RECORD_BYTES) > NAND_PAGE_SIZE_BYTES)
    {
        logger_note_light_payload_consistency_failure(logger);
        return LOG_ERR_BAD_ARGUMENT;
    }

    light_serialization_buffer_address =
        (uintptr_t)&logger->light_raw_page_buffer[payload_offset];

    logger_serialize_light_raw_record(&logger->light_raw_page_buffer[payload_offset],
                                      record);

    if (logger_light_raw_record_payload_is_valid(&logger->light_raw_page_buffer[payload_offset],
                                                 NULL) == 0U)
    {
        logger_note_light_payload_consistency_failure(logger);
        return LOG_ERR_BAD_ARGUMENT;
    }

    light_records_serialized++;

    logger->light_raw_records_in_page++;
    logger->light_raw_payload_bytes += LOG_LIGHT_RAW_RECORD_BYTES;
    light_records_counter_incremented++;

    if (!logger_light_raw_payload_is_consistent(logger))
    {
        logger_note_light_payload_consistency_failure(logger);
        return LOG_ERR_BAD_ARGUMENT;
    }

    if (logger->light_raw_payload_bytes >= LOG_LIGHT_RAW_MAX_PAYLOAD_BYTES)
    {
        status = logger_flush_light_raw_page(logger, timestamp_ms);
        if (status == LOG_OK)
        {
            light_records_appended++;
        }
        return status;
    }

    light_records_appended++;

    return LOG_OK;
}

LogStatus NANDLogger_Flush(NandLogger *logger, uint32_t timestamp_ms)
{
    return logger_flush_sensor_page(logger, timestamp_ms);
}

LogStatus NANDLogger_FlushLightRaw(NandLogger *logger, uint32_t timestamp_ms)
{
    return logger_flush_light_raw_page(logger, timestamp_ms);
}

LogStatus NANDLogger_FlushAll(NandLogger *logger, uint32_t timestamp_ms)
{
    LogStatus status;

    status = logger_flush_sensor_page(logger, timestamp_ms);
    if (status != LOG_OK)
    {
        return status;
    }

    return logger_flush_light_raw_page(logger, timestamp_ms);
}

static LogStatus logger_usb_send(const uint8_t *data, uint16_t len)
{
    uint32_t start_tick;
    uint8_t usb_status;

    if ((data == NULL) || (len == 0U))
    {
        return LOG_ERR_BAD_ARGUMENT;
    }

    start_tick = HAL_GetTick();
    usb_tx_last_length = len;

    while (1)
    {
        usb_cdc_tx_done = 0U;

        usb_status = CDC_Transmit_FS((uint8_t *)data, len);
        usb_tx_last_status = usb_status;

        if (usb_status == USBD_OK)
        {
            usb_tx_submit_count++;
            break;
        }

        if (usb_status != USBD_BUSY)
        {
            usb_tx_fail_count++;
            return LOG_ERR_USB;
        }

        usb_tx_busy_retry_count++;

        if ((HAL_GetTick() - start_tick) > USB_TX_TIMEOUT_MS)
        {
            usb_tx_timeout_count++;
            return LOG_ERR_USB;
        }
    }

    while (usb_cdc_tx_done == 0U)
    {
        if ((HAL_GetTick() - start_tick) > USB_TX_TIMEOUT_MS)
        {
            usb_tx_timeout_count++;
            return LOG_ERR_USB;
        }
    }

    usb_tx_complete_count++;

    return LOG_OK;
}

static bool logger_magic_is_valid(uint32_t magic)
{
    return (magic == LOG_MAGIC_SENSOR) ||
           (magic == LOG_MAGIC_AUDIO) ||
           (magic == LOG_MAGIC_LIGHT_RAW);
}

static bool logger_header_is_valid(const LogPageHeader *header)
{
    if (header == NULL)
    {
        return false;
    }

    if (!logger_magic_is_valid(header->magic))
    {
        return false;
    }

    if ((header->version != 1U) ||
        (header->header_size != LOG_HEADER_SIZE_BYTES) ||
        (header->payload_bytes > LOG_SENSOR_PAYLOAD_BYTES))
    {
        return false;
    }

    if ((header->magic == LOG_MAGIC_SENSOR) &&
        ((header->payload_bytes == 0U) ||
         ((header->payload_bytes % LOG_SENSOR_RECORD_BYTES) != 0U)))
    {
        return false;
    }

    if ((header->magic == LOG_MAGIC_AUDIO) &&
        ((header->payload_bytes == 0U) ||
         ((header->payload_bytes % sizeof(int16_t)) != 0U)))
    {
        return false;
    }

    if ((header->magic == LOG_MAGIC_LIGHT_RAW) &&
        ((header->payload_bytes == 0U) ||
         (header->payload_bytes > LOG_LIGHT_RAW_MAX_PAYLOAD_BYTES) ||
         ((header->payload_bytes % LOG_LIGHT_RAW_RECORD_BYTES) != 0U)))
    {
        return false;
    }

    return true;
}

static LogStatus logger_recover_written_pages(NandLogger *logger,
                                              uint32_t *written_pages)
{
    read_address_t addr;
    column_address_t column = 0U;
    LogPageHeader header;
    uint32_t logical_page = 0U;
    uint32_t max_pages;
    uint16_t good_block_index;
    uint8_t page_in_block;
    int nand_ret;

    if ((logger == NULL) || (written_pages == NULL))
    {
        return LOG_ERR_BAD_ARGUMENT;
    }

    *written_pages = 0U;

    if (logger->good_block_count == 0U)
    {
        return LOG_ERR_NO_GOOD_BLOCKS;
    }

    max_pages = (uint32_t)logger->good_block_count * NAND_PAGES_PER_BLOCK;

    while (logical_page < max_pages)
    {
        good_block_index = (uint16_t)(logical_page / NAND_PAGES_PER_BLOCK);
        page_in_block = (uint8_t)(logical_page % NAND_PAGES_PER_BLOCK);

        addr.block = logger->good_blocks[good_block_index];
        addr.page = page_in_block;
        addr.dummy = 0U;

        memset(&header, 0xFF, sizeof(header));

        nand_ret = spi_nand_page_read(addr,
                                      column,
                                      (uint8_t *)&header,
                                      sizeof(header));

        if (nand_ret != SPI_NAND_RET_OK)
        {
            return LOG_ERR_NAND;
        }

        if (!logger_header_is_valid(&header))
        {
            break;
        }

        logical_page++;
    }

    *written_pages = logical_page;
    logger->page_sequence = logical_page;
    logger->current_good_block_index = (uint16_t)(logical_page / NAND_PAGES_PER_BLOCK);
    logger->current_page_in_block = (uint8_t)(logical_page % NAND_PAGES_PER_BLOCK);

    return LOG_OK;
}

LogStatus NANDLogger_DownloadAll(NandLogger *logger)
{
    uint32_t total_pages;
    uint32_t logical_page;

    uint16_t good_block_index;
    uint8_t page_in_block;

    read_address_t addr;
    column_address_t column = 0U;

    LogStatus status;
    int nand_ret;

    const uint8_t start_marker[8] = {'L','O','G','S','T','A','R','T'};
    const uint8_t end_marker[8]   = {'L','O','G','E','N','D','!','!'};

    uint8_t total_pages_bytes[4];

    if (logger == NULL)
    {
        return LOG_ERR_BAD_ARGUMENT;
    }

    /*
     * Prima del download, salva eventuale pagina sensori parziale.
     *
     * Esempio:
     * se hai 37 record sensori in RAM ma non hai ancora raggiunto 102 record,
     * senza flush quei 37 record non sarebbero in NAND.
     */
    status = NANDLogger_FlushAll(logger, HAL_GetTick());
    if (status != LOG_OK)
    {
        return status;
    }

    /*
     * Dopo il flush, page_sequence rappresenta il numero totale
     * di pagine effettivamente scritte in NAND.
     */
    total_pages = logger->page_sequence;

    /*
     * page_sequence vive in RAM. Se la board e' ripartita tra acquisizione
     * e download, ricostruisci il conteggio leggendo gli header in NAND.
     */
    if (total_pages == 0U)
    {
        status = logger_recover_written_pages(logger, &total_pages);
        if (status != LOG_OK)
        {
            return status;
        }
    }

    /*
     * Invio marker iniziale: 8 byte = "LOGSTART".
     * Il PC userà questo per sincronizzarsi.
     */
    status = logger_usb_send(start_marker, sizeof(start_marker));
    App_UpdateDownloadLed();
    if (status != LOG_OK)
    {
        return status;
    }

    /*
     * Invio il numero totale di pagine come uint32 little-endian.
     */
    total_pages_bytes[0] = (uint8_t)(total_pages & 0xFFU);
    total_pages_bytes[1] = (uint8_t)((total_pages >> 8U) & 0xFFU);
    total_pages_bytes[2] = (uint8_t)((total_pages >> 16U) & 0xFFU);
    total_pages_bytes[3] = (uint8_t)((total_pages >> 24U) & 0xFFU);

    status = logger_usb_send(total_pages_bytes, sizeof(total_pages_bytes));
    App_UpdateDownloadLed();
    if (status != LOG_OK)
    {
        return status;
    }

    /*
     * Scarico tutte le pagine scritte.
     *
     * logical_page = 0,1,2,... viene convertita in:
     * - indice del blocco buono
     * - pagina dentro quel blocco
     */
    for (logical_page = 0U; logical_page < total_pages; logical_page++)
    {
        App_UpdateDownloadLed();

        good_block_index = (uint16_t)(logical_page / NAND_PAGES_PER_BLOCK);
        page_in_block = (uint8_t)(logical_page % NAND_PAGES_PER_BLOCK);

        if (good_block_index >= logger->good_block_count)
        {
            return LOG_ERR_FULL;
        }

        addr.block = logger->good_blocks[good_block_index];
        addr.page = page_in_block;
        addr.dummy = 0U;

        memset(logger_download_page_buffer, 0xFF, NAND_PAGE_SIZE_BYTES);

        /*
         * Lettura reale della pagina NAND.
         * Questa funzione esiste nel tuo SPI_NAND.h.
         */
        nand_ret = spi_nand_page_read(addr,
                                      column,
                                      logger_download_page_buffer,
                                      NAND_PAGE_SIZE_BYTES);

        if (nand_ret != SPI_NAND_RET_OK)
        {
            return LOG_ERR_NAND;
        }

        /*
         * Invio la pagina completa al PC.
         */
        status = logger_usb_send(logger_download_page_buffer,
                                 NAND_PAGE_SIZE_BYTES);
        App_UpdateDownloadLed();

        if (status != LOG_OK)
        {
            return status;
        }
    }

    /*
     * Marker finale: 8 byte = "LOGEND!!".
     */
    status = logger_usb_send(end_marker, sizeof(end_marker));
    App_UpdateDownloadLed();
    if (status != LOG_OK)
    {
        return status;
    }

    return LOG_OK;
}


/* -------------------------------------------------------------------------- */
/*                          Legacy helper functions                           */
/* -------------------------------------------------------------------------- */

void find_bad_blocks(uint16_t *bad_blocks)
{
    read_address_t blocco;
    bool is_bad_mark = true;
    int j = 0;

    if (bad_blocks == NULL)
    {
        return;
    }

    blocco.block = 0U;
    blocco.page = 0U;
    blocco.dummy = 0U;

    /*
     * NOTE:
     * Despite the name bad_blocks, this function fills the array with GOOD
     * blocks, because it stores block index only when !is_bad_mark.
     */
    for (int i = 0; i < 2048; i++)
    {
        blocco.block = (uint16_t)i;
        is_bad_mark = true;

        spi_nand_block_is_bad(blocco, &is_bad_mark);

        if (!is_bad_mark)
        {
            bad_blocks[j] = (uint16_t)i;
            j++;
        }
    }
}


void erase_good_blocks(uint8_t *bad_blocks_flag)
{
    read_address_t blocco;
    bool is_bad_mark = true;

    if (bad_blocks_flag == NULL)
    {
        return;
    }

    blocco.block = 0U;
    blocco.page = 0U;
    blocco.dummy = 0U;

    /*
     * bad_blocks_flag[i] = 1 means bad block.
     * bad_blocks_flag[i] = 0 means good block.
     */
    for (int i = 0; i < 2048; i++)
    {
        blocco.block = (uint16_t)i;
        is_bad_mark = true;

        spi_nand_block_is_bad(blocco, &is_bad_mark);

        if (is_bad_mark)
        {
            bad_blocks_flag[i] = 1U;
        }
        else
        {
            bad_blocks_flag[i] = 0U;
            spi_nand_block_erase(blocco);
        }
    }
}


/**
 * @brief Assembles one data record into a NAND page buffer.
 *
 * Packet layout, 40 bytes per sample:
 *
 * [0]      hh
 * [1]      mm
 * [2]      ss
 * [3..4]   sss, uint16 little-endian
 * [5..10]  accelerometer raw bytes
 * [11..16] gyroscope raw bytes
 * [17..38] reserved light area, zero-filled by the new AS7341 pipeline
 * [39]     reserved
 */
void write_packet(uint16_t sample_index,
                  Time_Struct timestamp,
                  uint8_t *accelerometer,
                  uint8_t *gyroscope,
                  uint8_t *light_raw,
                  uint8_t *packet_buffer)
{
    uint16_t base;

    if ((accelerometer == NULL) ||
        (gyroscope == NULL) ||
        (light_raw == NULL) ||
        (packet_buffer == NULL))
    {
        return;
    }

    base = sample_index * STRIDE_BYTES_PER_SAMPLE;

    /* Timestamp */
    packet_buffer[base + 0U] = timestamp.hh;
    packet_buffer[base + 1U] = timestamp.mm;
    packet_buffer[base + 2U] = timestamp.ss;

    packet_buffer[base + 3U] = (uint8_t)(timestamp.sss & 0xFFU);
    packet_buffer[base + 4U] = (uint8_t)((timestamp.sss >> 8U) & 0xFFU);

    /* Accelerometer, 6 bytes */
    packet_buffer[base + 5U]  = accelerometer[0];
    packet_buffer[base + 6U]  = accelerometer[1];
    packet_buffer[base + 7U]  = accelerometer[2];
    packet_buffer[base + 8U]  = accelerometer[3];
    packet_buffer[base + 9U]  = accelerometer[4];
    packet_buffer[base + 10U] = accelerometer[5];

    /* Gyroscope, 6 bytes */
    packet_buffer[base + 11U] = gyroscope[0];
    packet_buffer[base + 12U] = gyroscope[1];
    packet_buffer[base + 13U] = gyroscope[2];
    packet_buffer[base + 14U] = gyroscope[3];
    packet_buffer[base + 15U] = gyroscope[4];
    packet_buffer[base + 16U] = gyroscope[5];

    /* Legacy light area kept for 40-byte record compatibility. */
    for (uint8_t i = 0U; i < 22U; i++)
    {
        packet_buffer[base + 17U + i] = light_raw[i];
    }

    /* Reserved byte */
    packet_buffer[base + 39U] = 0x00U;
}


/*
 * Legacy audio writer.
 *
 * This function is kept only for backward compatibility.
 * New code should use NANDLogger_AppendAudioBuffer().
 */
void write_audio_page(int16_t *audio_buffer, uint32_t audio_samples)
{
    uint32_t audio_bytes;

    if (audio_buffer == NULL)
    {
        return;
    }

    audio_bytes = audio_samples * sizeof(int16_t);

    if (audio_bytes > (4096U - 8U))
    {
        return;
    }

    if (audio_pagina_scritta >= 64U)
    {
        audio_pagina_scritta = 0U;
        audio_b++;
    }

    if (audio_b >= 2048U)
    {
        return;
    }

    memset(audio_NAND_packet, 0xFF, sizeof(audio_NAND_packet));

    audio_NAND_packet[0] = 'A';
    audio_NAND_packet[1] = 'U';
    audio_NAND_packet[2] = 'D';
    audio_NAND_packet[3] = '0';

    audio_NAND_packet[4] = (uint8_t)(audio_bytes & 0xFFU);
    audio_NAND_packet[5] = (uint8_t)((audio_bytes >> 8U) & 0xFFU);
    audio_NAND_packet[6] = (uint8_t)((audio_bytes >> 16U) & 0xFFU);
    audio_NAND_packet[7] = (uint8_t)((audio_bytes >> 24U) & 0xFFU);

    memcpy(&audio_NAND_packet[8],
           (uint8_t *)audio_buffer,
           audio_bytes);

    audio_blocco_scritto = bad_blocks[audio_b];

    audio_blocco.block = audio_blocco_scritto;
    audio_blocco.page = audio_pagina_scritta;
    audio_blocco.dummy = 0U;

    audio_colonna = 0U;

    spi_nand_page_program(audio_blocco,
                          audio_colonna,
                          audio_NAND_packet,
                          4096U);

    audio_pagina_scritta++;
}


/* -------------------------------------------------------------------------- */
/*                   Legacy functions: safe compatibility stubs               */
/* -------------------------------------------------------------------------- */

/*
 * These functions were declared in Memory_operations.h.
 * If your new main.c no longer calls them, they are not used.
 * They are kept here to avoid linker errors during the transition.
 */

void erase_memory(void)
{
    /*
     * Old-style erase using bad_blocks2 flags.
     * Prefer NANDLogger_EraseAllGoodBlocks() in new code.
     */
    erase_good_blocks(bad_blocks2);
}


void write_memory(void)
{
    /*
     * Old implementation not used by the new logger.
     *
     * New code should call:
     *
     *   NANDLogger_AppendSensorRecord(...)
     *
     * This function is intentionally left empty for compatibility.
     */
}


void read_memory_and_transmit(void)
{
    /*
     * Old implementation not used by the new logger.
     *
     * New code should call:
     *
     *   NANDLogger_DownloadAll(...)
     *
     * This function is intentionally left empty for compatibility.
     */
}


NAND_info read_memory(int b_index,
                      NAND_info indice,
                      uint16_t *blocco_letto,
                      uint8_t *pagina_letta,
                      uint16_t good_blocks[2048],
                      uint8_t *data_letto)
{
    /*
     * Compatibility stub.
     *
     * Real implementation requires the correct SPI_NAND.c read API.
     */
    (void)b_index;
    (void)good_blocks;
    (void)data_letto;

    if (blocco_letto != NULL)
    {
        *blocco_letto = indice.blocco_scritto;
    }

    if (pagina_letta != NULL)
    {
        *pagina_letta = indice.pagina_scritta;
    }

    return indice;
}


void write_info(NAND_info segnalibro,
                uint16_t good_blocks[2048])
{
    /*
     * Compatibility stub.
     */
    (void)segnalibro;
    (void)good_blocks;
}


NAND_info read_info(uint16_t good_blocks[2048])
{
    NAND_info info;

    (void)good_blocks;

    info.blocco_scritto = 0U;
    info.pagina_scritta = 0U;
    info.b = 0;

    return info;
}

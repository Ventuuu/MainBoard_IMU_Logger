#include "ble_sync_protocol.h"

#include <string.h>

static void ble_sync_put_u16_le(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static uint16_t ble_sync_get_u16_le(const uint8_t *src)
{
    return (uint16_t)((uint16_t)src[0] |
                      ((uint16_t)src[1] << 8U));
}

uint16_t BleSync_Crc16CcittFalse(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;

    if ((data == NULL) && (length != 0U))
    {
        return crc;
    }

    for (size_t i = 0U; i < length; i++)
    {
        crc ^= (uint16_t)data[i] << 8U;
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            crc = ((crc & 0x8000U) != 0U) ?
                  (uint16_t)((crc << 1U) ^ 0x1021U) :
                  (uint16_t)(crc << 1U);
        }
    }

    return crc;
}

uint32_t BleSync_Crc32IsoHdlcUpdate(uint32_t state,
                                   const uint8_t *data,
                                   size_t length)
{
    if ((data == NULL) && (length != 0U))
    {
        return state;
    }

    for (size_t i = 0U; i < length; i++)
    {
        state ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            state = ((state & 1U) != 0U) ?
                    ((state >> 1U) ^ 0xEDB88320UL) :
                    (state >> 1U);
        }
    }

    return state;
}

uint32_t BleSync_Crc32IsoHdlc(const uint8_t *data, size_t length)
{
    return BleSync_Crc32IsoHdlcUpdate(0xFFFFFFFFUL, data, length) ^
           0xFFFFFFFFUL;
}

size_t BleSync_EncodeFrame(uint8_t message_type,
                           uint16_t message_sequence,
                           const uint8_t *payload,
                           uint16_t payload_length,
                           uint8_t *output,
                           size_t output_capacity)
{
    size_t frame_length = BLE_SYNC_FRAME_PREFIX_BYTES +
                          payload_length + BLE_SYNC_FRAME_CRC_BYTES;
    uint16_t crc;

    if ((output == NULL) ||
        ((payload == NULL) && (payload_length != 0U)) ||
        (payload_length > BLE_SYNC_MAX_PAYLOAD_BYTES) ||
        (output_capacity < frame_length))
    {
        return 0U;
    }

    output[0] = BLE_SYNC_SOF1;
    output[1] = BLE_SYNC_SOF2;
    output[2] = BLE_SYNC_PROTOCOL_VERSION;
    output[3] = message_type;
    ble_sync_put_u16_le(&output[4], payload_length);
    ble_sync_put_u16_le(&output[6], message_sequence);

    if (payload_length != 0U)
    {
        memcpy(&output[8], payload, payload_length);
    }

    crc = BleSync_Crc16CcittFalse(&output[2],
                                  BLE_SYNC_FRAME_BODY_HEADER_BYTES +
                                  payload_length);
    ble_sync_put_u16_le(&output[8U + payload_length], crc);

    return frame_length;
}

bool BleSync_VerifyEncodedFrameCrc(const uint8_t *frame, size_t frame_length)
{
    uint16_t payload_length;
    size_t expected_length;
    uint16_t expected_crc;
    uint16_t received_crc;

    if ((frame == NULL) || (frame_length < 10U) ||
        (frame[0] != BLE_SYNC_SOF1) || (frame[1] != BLE_SYNC_SOF2))
    {
        return false;
    }

    payload_length = ble_sync_get_u16_le(&frame[4]);
    expected_length = BLE_SYNC_FRAME_PREFIX_BYTES + payload_length +
                      BLE_SYNC_FRAME_CRC_BYTES;
    if ((payload_length > BLE_SYNC_MAX_PAYLOAD_BYTES) ||
        (frame_length != expected_length))
    {
        return false;
    }

    expected_crc = BleSync_Crc16CcittFalse(
            &frame[2], BLE_SYNC_FRAME_BODY_HEADER_BYTES + payload_length);
    received_crc = ble_sync_get_u16_le(&frame[8U + payload_length]);
    return expected_crc == received_crc;
}

void BleSync_ResetParser(BleSyncFrameParser *parser)
{
    if (parser == NULL)
    {
        return;
    }

    memset(parser, 0, sizeof(*parser));
    parser->state = BLE_SYNC_PARSER_WAIT_SOF1;
}

BleSyncParseResult BleSync_ParseByte(BleSyncFrameParser *parser,
                                     uint8_t byte,
                                     uint32_t now_ms,
                                     BleSyncFrame *frame)
{
    uint16_t payload_length;
    uint16_t expected_crc;
    uint8_t protocol_version;

    if ((parser == NULL) || (frame == NULL))
    {
        return BLE_SYNC_PARSE_LENGTH_ERROR;
    }

    parser->last_byte_ms = now_ms;

    switch (parser->state)
    {
        case BLE_SYNC_PARSER_WAIT_SOF1:
            if (byte == BLE_SYNC_SOF1)
            {
                parser->state = BLE_SYNC_PARSER_WAIT_SOF2;
            }
            return BLE_SYNC_PARSE_NONE;

        case BLE_SYNC_PARSER_WAIT_SOF2:
            if (byte == BLE_SYNC_SOF2)
            {
                parser->body_index = 0U;
                parser->expected_body_bytes =
                        BLE_SYNC_FRAME_BODY_HEADER_BYTES;
                parser->state = BLE_SYNC_PARSER_READ_BODY;
            }
            else if (byte != BLE_SYNC_SOF1)
            {
                parser->state = BLE_SYNC_PARSER_WAIT_SOF1;
            }
            return BLE_SYNC_PARSE_NONE;

        case BLE_SYNC_PARSER_READ_BODY:
            if (parser->body_index >= sizeof(parser->body))
            {
                BleSync_ResetParser(parser);
                return BLE_SYNC_PARSE_LENGTH_ERROR;
            }

            parser->body[parser->body_index++] = byte;
            if (parser->body_index == BLE_SYNC_FRAME_BODY_HEADER_BYTES)
            {
                payload_length = ble_sync_get_u16_le(&parser->body[2]);
                if (payload_length > BLE_SYNC_MAX_PAYLOAD_BYTES)
                {
                    BleSync_ResetParser(parser);
                    return BLE_SYNC_PARSE_LENGTH_ERROR;
                }
                parser->expected_body_bytes =
                        BLE_SYNC_FRAME_BODY_HEADER_BYTES + payload_length;
            }

            if (parser->body_index == parser->expected_body_bytes)
            {
                parser->state = BLE_SYNC_PARSER_READ_CRC_LOW;
            }
            return BLE_SYNC_PARSE_NONE;

        case BLE_SYNC_PARSER_READ_CRC_LOW:
            parser->received_crc = byte;
            parser->state = BLE_SYNC_PARSER_READ_CRC_HIGH;
            return BLE_SYNC_PARSE_NONE;

        case BLE_SYNC_PARSER_READ_CRC_HIGH:
            parser->received_crc |= (uint16_t)byte << 8U;
            expected_crc = BleSync_Crc16CcittFalse(parser->body,
                                                   parser->expected_body_bytes);
            if (expected_crc != parser->received_crc)
            {
                BleSync_ResetParser(parser);
                return BLE_SYNC_PARSE_CRC_ERROR;
            }

            protocol_version = parser->body[0];
            if (protocol_version != BLE_SYNC_PROTOCOL_VERSION)
            {
                BleSync_ResetParser(parser);
                return BLE_SYNC_PARSE_VERSION_ERROR;
            }

            frame->protocol_version = protocol_version;
            frame->message_type = parser->body[1];
            frame->payload_length = ble_sync_get_u16_le(&parser->body[2]);
            frame->message_sequence = ble_sync_get_u16_le(&parser->body[4]);
            if (frame->payload_length != 0U)
            {
                memcpy(frame->payload,
                       &parser->body[BLE_SYNC_FRAME_BODY_HEADER_BYTES],
                       frame->payload_length);
            }
            BleSync_ResetParser(parser);
            return BLE_SYNC_PARSE_FRAME;

        default:
            BleSync_ResetParser(parser);
            return BLE_SYNC_PARSE_LENGTH_ERROR;
    }
}

bool BleSync_ParserHasTimedOut(const BleSyncFrameParser *parser,
                               uint32_t now_ms,
                               uint32_t timeout_ms)
{
    if ((parser == NULL) ||
        (parser->state == BLE_SYNC_PARSER_WAIT_SOF1))
    {
        return false;
    }

    return (now_ms - parser->last_byte_ms) >= timeout_ms;
}

void BleSync_BuildMetadata(BleSyncMetadataV1 *record,
                           uint32_t metadata_sequence,
                           uint32_t log_generation,
                           bool ack_valid,
                           uint32_t acked_through_page_sequence)
{
    if (record == NULL)
    {
        return;
    }

    memset(record, 0, sizeof(*record));
    record->magic = BLE_SYNC_METADATA_MAGIC;
    record->version = BLE_SYNC_METADATA_VERSION;
    record->record_size = (uint8_t)sizeof(*record);
    record->flags = ack_valid ? BLE_SYNC_METADATA_FLAG_ACK_VALID : 0U;
    record->metadata_sequence = metadata_sequence;
    record->log_generation = log_generation;
    record->acked_through_page_sequence = acked_through_page_sequence;
    record->crc32 = BleSync_Crc32IsoHdlc((const uint8_t *)record,
                                         offsetof(BleSyncMetadataV1, crc32));
}

bool BleSync_MetadataIsValid(const BleSyncMetadataV1 *record)
{
    uint32_t crc;

    if ((record == NULL) ||
        (record->magic != BLE_SYNC_METADATA_MAGIC) ||
        (record->version != BLE_SYNC_METADATA_VERSION) ||
        (record->record_size != sizeof(*record)) ||
        ((record->flags & ~BLE_SYNC_METADATA_FLAG_ACK_VALID) != 0U) ||
        (record->log_generation == 0U))
    {
        return false;
    }

    crc = BleSync_Crc32IsoHdlc((const uint8_t *)record,
                               offsetof(BleSyncMetadataV1, crc32));
    return crc == record->crc32;
}

bool BleSync_MetadataSequenceIsNewer(uint32_t candidate,
                                     uint32_t reference)
{
    return (int32_t)(candidate - reference) > 0;
}

bool BleSync_SelectLatestMetadata(const BleSyncMetadataV1 *records,
                                  size_t record_count,
                                  BleSyncMetadataV1 *latest)
{
    bool found = false;

    if ((records == NULL) || (latest == NULL))
    {
        return false;
    }

    for (size_t i = 0U; i < record_count; i++)
    {
        if (!BleSync_MetadataIsValid(&records[i]))
        {
            continue;
        }
        if (!found || BleSync_MetadataSequenceIsNewer(
                records[i].metadata_sequence,
                latest->metadata_sequence))
        {
            *latest = records[i];
            found = true;
        }
    }

    return found;
}

uint32_t BleSync_RunProtocolSelfTests(void)
{
    static const uint8_t vector[] = "123456789";
    uint8_t frame_a[BLE_SYNC_MAX_FRAME_BYTES];
    uint8_t frame_b[BLE_SYNC_MAX_FRAME_BYTES];
    uint8_t oversized_header[] = {
        BLE_SYNC_SOF1, BLE_SYNC_SOF2, BLE_SYNC_PROTOCOL_VERSION,
        BLE_MSG_SYNC_START,
        (uint8_t)((BLE_SYNC_MAX_PAYLOAD_BYTES + 1U) & 0xFFU),
        (uint8_t)((BLE_SYNC_MAX_PAYLOAD_BYTES + 1U) >> 8U),
        0U, 0U
    };
    uint8_t payload[] = {0x11U, 0x22U, 0x33U};
    BleSyncFrameParser parser;
    BleSyncFrame parsed;
    BleSyncMetadataV1 metadata;
    BleSyncMetadataV1 metadata_candidates[3];
    BleSyncMetadataV1 latest_metadata;
    size_t frame_a_length;
    size_t frame_b_length;
    uint32_t failures = 0U;
    uint32_t parsed_frames = 0U;
    BleSyncParseResult result = BLE_SYNC_PARSE_NONE;

    if (BleSync_Crc16CcittFalse(vector, sizeof(vector) - 1U) != 0x29B1U)
    {
        failures |= (1UL << 0);
    }
    if (BleSync_Crc32IsoHdlc(vector, sizeof(vector) - 1U) != 0xCBF43926UL)
    {
        failures |= (1UL << 1);
    }

    frame_a_length = BleSync_EncodeFrame(BLE_MSG_PAGE_END, 7U,
                                         payload, sizeof(payload),
                                         frame_a, sizeof(frame_a));
    frame_b_length = BleSync_EncodeFrame(BLE_MSG_SYNC_START, 8U,
                                         NULL, 0U,
                                         frame_b, sizeof(frame_b));
    if ((frame_a_length == 0U) ||
        !BleSync_VerifyEncodedFrameCrc(frame_a, frame_a_length))
    {
        failures |= (1UL << 2);
    }

    BleSync_ResetParser(&parser);
    for (size_t i = 0U; i < frame_a_length; i++)
    {
        result = BleSync_ParseByte(&parser, frame_a[i], (uint32_t)i, &parsed);
    }
    if ((result != BLE_SYNC_PARSE_FRAME) ||
        (parsed.message_type != BLE_MSG_PAGE_END) ||
        (parsed.message_sequence != 7U) ||
        (parsed.payload_length != sizeof(payload)) ||
        (memcmp(parsed.payload, payload, sizeof(payload)) != 0))
    {
        failures |= (1UL << 3);
    }

    BleSync_ResetParser(&parser);
    result = BLE_SYNC_PARSE_NONE;
    for (size_t i = 0U; i < frame_a_length / 2U; i++)
    {
        result = BleSync_ParseByte(&parser, frame_a[i], (uint32_t)i, &parsed);
    }
    if (result != BLE_SYNC_PARSE_NONE)
    {
        failures |= (1UL << 4);
    }
    for (size_t i = frame_a_length / 2U; i < frame_a_length; i++)
    {
        result = BleSync_ParseByte(&parser, frame_a[i], (uint32_t)i, &parsed);
    }
    if (result != BLE_SYNC_PARSE_FRAME)
    {
        failures |= (1UL << 4);
    }

    BleSync_ResetParser(&parser);
    parsed_frames = 0U;
    for (size_t i = 0U; i < frame_a_length + frame_b_length; i++)
    {
        uint8_t byte = (i < frame_a_length) ?
                       frame_a[i] : frame_b[i - frame_a_length];
        if (BleSync_ParseByte(&parser, byte, (uint32_t)i, &parsed) ==
            BLE_SYNC_PARSE_FRAME)
        {
            parsed_frames++;
        }
    }
    if (parsed_frames != 2U)
    {
        failures |= (1UL << 5);
    }

    BleSync_ResetParser(&parser);
    (void)BleSync_ParseByte(&parser, 0x00U, 0U, &parsed);
    (void)BleSync_ParseByte(&parser, 0x55U, 1U, &parsed);
    for (size_t i = 0U; i < frame_b_length; i++)
    {
        result = BleSync_ParseByte(&parser, frame_b[i],
                                   (uint32_t)(i + 2U), &parsed);
    }
    if (result != BLE_SYNC_PARSE_FRAME)
    {
        failures |= (1UL << 6);
    }

    frame_a[frame_a_length - 1U] ^= 0x01U;
    BleSync_ResetParser(&parser);
    for (size_t i = 0U; i < frame_a_length; i++)
    {
        result = BleSync_ParseByte(&parser, frame_a[i], (uint32_t)i, &parsed);
    }
    if (result != BLE_SYNC_PARSE_CRC_ERROR)
    {
        failures |= (1UL << 7);
    }

    BleSync_ResetParser(&parser);
    result = BLE_SYNC_PARSE_NONE;
    for (size_t i = 0U; i < sizeof(oversized_header); i++)
    {
        result = BleSync_ParseByte(&parser, oversized_header[i],
                                   (uint32_t)i, &parsed);
    }
    if (result != BLE_SYNC_PARSE_LENGTH_ERROR)
    {
        failures |= (1UL << 8);
    }

    frame_b[2] = BLE_SYNC_PROTOCOL_VERSION + 1U;
    {
        uint16_t crc = BleSync_Crc16CcittFalse(&frame_b[2],
                                               BLE_SYNC_FRAME_BODY_HEADER_BYTES);
        ble_sync_put_u16_le(&frame_b[8], crc);
    }
    BleSync_ResetParser(&parser);
    for (size_t i = 0U; i < frame_b_length; i++)
    {
        result = BleSync_ParseByte(&parser, frame_b[i], (uint32_t)i, &parsed);
    }
    if (result != BLE_SYNC_PARSE_VERSION_ERROR)
    {
        failures |= (1UL << 9);
    }

    BleSync_BuildMetadata(&metadata, 5U, 7U, true, 42U);
    if (!BleSync_MetadataIsValid(&metadata) ||
        (metadata.log_generation != 7U) ||
        (metadata.acked_through_page_sequence != 42U))
    {
        failures |= (1UL << 10);
    }
    metadata.crc32 ^= 1U;
    if (BleSync_MetadataIsValid(&metadata))
    {
        failures |= (1UL << 11);
    }
    if (!BleSync_MetadataSequenceIsNewer(6U, 5U))
    {
        failures |= (1UL << 12);
    }
    BleSync_BuildMetadata(&metadata, 1U, 1U, false, 0U);
    if (!BleSync_MetadataIsValid(&metadata) ||
        ((metadata.flags & BLE_SYNC_METADATA_FLAG_ACK_VALID) != 0U))
    {
        failures |= (1UL << 13);
    }

    BleSync_ResetParser(&parser);
    (void)BleSync_ParseByte(&parser, BLE_SYNC_SOF1, 100U, &parsed);
    if (!BleSync_ParserHasTimedOut(&parser, 1100U, 1000U))
    {
        failures |= (1UL << 14);
    }

    BleSync_BuildMetadata(&metadata_candidates[0], 8U, 3U, true, 20U);
    BleSync_BuildMetadata(&metadata_candidates[1], 10U, 4U, true, 30U);
    BleSync_BuildMetadata(&metadata_candidates[2], 9U, 3U, true, 25U);
    metadata_candidates[2].crc32 ^= 1U;
    if (!BleSync_SelectLatestMetadata(metadata_candidates, 3U,
                                      &latest_metadata) ||
        (latest_metadata.metadata_sequence != 10U) ||
        (latest_metadata.log_generation != 4U) ||
        (latest_metadata.acked_through_page_sequence != 30U))
    {
        failures |= (1UL << 15);
    }

    return failures;
}

#if defined(BLE_SYNC_PROTOCOL_STANDALONE_MAIN)
#include <stdio.h>
int main(void)
{
    uint32_t failures = BleSync_RunProtocolSelfTests();
    printf("BLE sync protocol self-test failures: 0x%08lX\n",
           (unsigned long)failures);
    return failures == 0U ? 0 : 1;
}
#endif

#ifndef INC_BLE_SYNC_PROTOCOL_H_
#define INC_BLE_SYNC_PROTOCOL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BLE_SYNC_PROTOCOL_VERSION       1U
#define BLE_SYNC_SOF1                   0xA5U
#define BLE_SYNC_SOF2                   0x5AU

#define BLE_MSG_SYNC_START              0x10U
#define BLE_MSG_SYNC_STATUS             0x11U
#define BLE_MSG_PAGE_BEGIN              0x20U
#define BLE_MSG_PAGE_DATA               0x21U
#define BLE_MSG_PAGE_END                0x22U
#define BLE_MSG_ACK_THROUGH             0x30U
#define BLE_MSG_NACK_PAGE               0x31U
#define BLE_MSG_SYNC_COMPLETE           0x40U
#define BLE_MSG_SYNC_ABORT              0x41U
#define BLE_MSG_ERROR                   0x7FU

#define BLE_SYNC_CHUNK_DATA_MAX         128U
#define BLE_SYNC_MAX_PAYLOAD_BYTES      (16U + BLE_SYNC_CHUNK_DATA_MAX)
#define BLE_SYNC_FRAME_PREFIX_BYTES     8U
#define BLE_SYNC_FRAME_CRC_BYTES        2U
#define BLE_SYNC_FRAME_BODY_HEADER_BYTES 6U
#define BLE_SYNC_MAX_FRAME_BYTES \
    (BLE_SYNC_FRAME_PREFIX_BYTES + BLE_SYNC_MAX_PAYLOAD_BYTES + \
     BLE_SYNC_FRAME_CRC_BYTES)

#define BLE_SYNC_METADATA_MAGIC         0x414E5953UL /* "SYNA" */
#define BLE_SYNC_METADATA_VERSION       1U
#define BLE_SYNC_METADATA_FLAG_ACK_VALID (1U << 0)

typedef struct __attribute__((packed))
{
    uint32_t magic;
    uint8_t version;
    uint8_t record_size;
    uint16_t flags;
    uint32_t metadata_sequence;
    uint32_t log_generation;
    uint32_t acked_through_page_sequence;
    uint32_t reserved0;
    uint32_t reserved1;
    uint32_t crc32;
} BleSyncMetadataV1;

_Static_assert(sizeof(BleSyncMetadataV1) == 32U,
               "BleSyncMetadataV1 must remain 32 bytes");
_Static_assert(offsetof(BleSyncMetadataV1, crc32) == 28U,
               "BleSyncMetadataV1 CRC offset must remain 28");
_Static_assert(BLE_SYNC_MAX_PAYLOAD_BYTES >= (16U + BLE_SYNC_CHUNK_DATA_MAX),
               "BLE sync payload buffer is too small");

typedef struct
{
    uint8_t protocol_version;
    uint8_t message_type;
    uint16_t payload_length;
    uint16_t message_sequence;
    uint8_t payload[BLE_SYNC_MAX_PAYLOAD_BYTES];
} BleSyncFrame;

typedef enum
{
    BLE_SYNC_PARSE_NONE = 0,
    BLE_SYNC_PARSE_FRAME,
    BLE_SYNC_PARSE_CRC_ERROR,
    BLE_SYNC_PARSE_LENGTH_ERROR,
    BLE_SYNC_PARSE_VERSION_ERROR
} BleSyncParseResult;

typedef enum
{
    BLE_SYNC_PARSER_WAIT_SOF1 = 0,
    BLE_SYNC_PARSER_WAIT_SOF2,
    BLE_SYNC_PARSER_READ_BODY,
    BLE_SYNC_PARSER_READ_CRC_LOW,
    BLE_SYNC_PARSER_READ_CRC_HIGH
} BleSyncParserState;

typedef struct
{
    BleSyncParserState state;
    uint8_t body[BLE_SYNC_FRAME_BODY_HEADER_BYTES +
                 BLE_SYNC_MAX_PAYLOAD_BYTES];
    uint16_t body_index;
    uint16_t expected_body_bytes;
    uint16_t received_crc;
    uint32_t last_byte_ms;
} BleSyncFrameParser;

uint16_t BleSync_Crc16CcittFalse(const uint8_t *data, size_t length);
uint32_t BleSync_Crc32IsoHdlc(const uint8_t *data, size_t length);
uint32_t BleSync_Crc32IsoHdlcUpdate(uint32_t state,
                                   const uint8_t *data,
                                   size_t length);

size_t BleSync_EncodeFrame(uint8_t message_type,
                           uint16_t message_sequence,
                           const uint8_t *payload,
                           uint16_t payload_length,
                           uint8_t *output,
                           size_t output_capacity);
bool BleSync_VerifyEncodedFrameCrc(const uint8_t *frame, size_t frame_length);

void BleSync_ResetParser(BleSyncFrameParser *parser);
BleSyncParseResult BleSync_ParseByte(BleSyncFrameParser *parser,
                                     uint8_t byte,
                                     uint32_t now_ms,
                                     BleSyncFrame *frame);
bool BleSync_ParserHasTimedOut(const BleSyncFrameParser *parser,
                               uint32_t now_ms,
                               uint32_t timeout_ms);

void BleSync_BuildMetadata(BleSyncMetadataV1 *record,
                           uint32_t metadata_sequence,
                           uint32_t log_generation,
                           bool ack_valid,
                           uint32_t acked_through_page_sequence);
bool BleSync_MetadataIsValid(const BleSyncMetadataV1 *record);
bool BleSync_MetadataSequenceIsNewer(uint32_t candidate,
                                     uint32_t reference);
bool BleSync_SelectLatestMetadata(const BleSyncMetadataV1 *records,
                                  size_t record_count,
                                  BleSyncMetadataV1 *latest);

uint32_t BleSync_RunProtocolSelfTests(void);

#endif /* INC_BLE_SYNC_PROTOCOL_H_ */

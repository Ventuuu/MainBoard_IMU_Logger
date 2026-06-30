#ifndef INC_BLE_SYNC_H_
#define INC_BLE_SYNC_H_

#include <stdint.h>

#include "Memory_operations.h"
#include "ble_sync_protocol.h"

#define BLE_SYNC_START_TIMEOUT_MS          30000U
#define BLE_SYNC_ACK_TIMEOUT_MS             3000U
#define BLE_SYNC_FRAME_TIMEOUT_MS           1000U
#define BLE_SYNC_UART_TX_TIMEOUT_MS         1000U
#define BLE_SYNC_MAX_PAGE_RETRIES              3U
#define BLE_SYNC_ACK_PERSIST_INTERVAL_PAGES    8U
#define BLE_SYNC_METADATA_RESERVED_BLOCKS      2U
#define BLE_SYNC_SCAN_PAGES_PER_PROCESS         8U

#ifndef BLE_SYNC_ENABLE_BOOT_SELF_TESTS
#define BLE_SYNC_ENABLE_BOOT_SELF_TESTS         0U
#endif

typedef enum
{
    BLE_SYNC_IDLE = 0,
    BLE_SYNC_REQUESTED,
    BLE_SYNC_PREPARE,
    BLE_SYNC_WAIT_START,
    BLE_SYNC_SEND_STATUS,
    BLE_SYNC_FIND_NEXT_PAGE,
    BLE_SYNC_SEND_PAGE_BEGIN,
    BLE_SYNC_SEND_PAGE_DATA,
    BLE_SYNC_SEND_PAGE_END,
    BLE_SYNC_WAIT_ACK,
    BLE_SYNC_PERSIST_ACK,
    BLE_SYNC_COMPLETE,
    BLE_SYNC_ABORT,
    BLE_SYNC_ERROR
} BleSyncState;

typedef enum
{
    BLE_SYNC_ABORT_NONE = 0,
    BLE_SYNC_ABORT_USER = 1,
    BLE_SYNC_ABORT_START_TIMEOUT = 2,
    BLE_SYNC_ABORT_ACK_TIMEOUT = 3,
    BLE_SYNC_ABORT_MAXIMUM_RETRIES = 4,
    BLE_SYNC_ABORT_CONNECTION_LOST = 5,
    BLE_SYNC_ABORT_UART_ERROR = 6,
    BLE_SYNC_ABORT_PROTOCOL_ERROR = 7,
    BLE_SYNC_ABORT_METADATA_PERSISTENCE_ERROR = 8,
    BLE_SYNC_ABORT_USB_PREEMPTION = 9,
    BLE_SYNC_ABORT_NAND_ERROR = 10
} BleSyncAbortReason;

typedef enum
{
    BLE_SYNC_NACK_PAGE_CRC = 1,
    BLE_SYNC_NACK_FRAME_INCOMPLETE = 2,
    BLE_SYNC_NACK_LOCAL_SAVE_FAILED = 3,
    BLE_SYNC_NACK_GENERIC = 4
} BleSyncNackReason;

typedef enum
{
    BLE_SYNC_ERROR_NONE = 0,
    BLE_SYNC_ERROR_FRAME_CRC = 1,
    BLE_SYNC_ERROR_FRAME_LENGTH = 2,
    BLE_SYNC_ERROR_PROTOCOL_VERSION = 3,
    BLE_SYNC_ERROR_UNEXPECTED_MESSAGE = 4,
    BLE_SYNC_ERROR_INVALID_ACK = 5,
    BLE_SYNC_ERROR_INVALID_NACK = 6,
    BLE_SYNC_ERROR_NAND_READ = 7,
    BLE_SYNC_ERROR_NAND_FLUSH = 8,
    BLE_SYNC_ERROR_UART = 9,
    BLE_SYNC_ERROR_METADATA = 10,
    BLE_SYNC_ERROR_RX_OVERFLOW = 11,
    BLE_SYNC_ERROR_FRAME_TIMEOUT = 12
} BleSyncError;

typedef struct
{
    uint8_t last_rx_message_type;
    uint8_t last_tx_message_type;
    uint16_t last_rx_message_sequence;
    uint16_t last_tx_message_sequence;
    uint8_t current_page_magic[4];
    uint8_t current_page_header_size;
    uint32_t current_page_payload_bytes;
    uint32_t current_page_logical_bytes;
    uint32_t current_page_crc32;
    uint32_t last_ack_generation;
    uint32_t last_ack_page_sequence;
} BleSyncLatestDiagnostics;

extern volatile BleSyncState ble_sync_state;
extern volatile uint8_t ble_sync_requested;
extern volatile uint8_t ble_sync_abort_requested;
extern volatile uint8_t ble_sync_active;

extern volatile uint32_t ble_sync_sessions_started;
extern volatile uint32_t ble_sync_sessions_completed;
extern volatile uint32_t ble_sync_sessions_aborted;

extern volatile uint32_t ble_sync_pages_planned;
extern volatile uint32_t ble_sync_pages_started;
extern volatile uint32_t ble_sync_pages_sent;
extern volatile uint32_t ble_sync_pages_acked;
extern volatile uint32_t ble_sync_pages_retransmitted;

extern volatile uint32_t ble_sync_frames_tx;
extern volatile uint32_t ble_sync_frames_rx;
extern volatile uint32_t ble_sync_frame_crc_errors;
extern volatile uint32_t ble_sync_frame_length_errors;
extern volatile uint32_t ble_sync_protocol_version_errors;
extern volatile uint32_t ble_sync_uart_errors;

extern volatile uint32_t ble_sync_ack_timeouts;
extern volatile uint32_t ble_sync_invalid_acks;
extern volatile uint32_t ble_sync_duplicate_acks;
extern volatile uint32_t ble_sync_nacks_received;

extern volatile uint32_t ble_sync_log_generation;
extern volatile uint8_t ble_sync_ack_valid;
extern volatile uint32_t ble_sync_acked_through_ram;
extern volatile uint32_t ble_sync_acked_through_persisted;
extern volatile uint32_t ble_sync_high_watermark;
extern volatile uint32_t ble_sync_high_watermark_physical_page;
extern volatile uint32_t ble_sync_current_page_sequence;
extern volatile uint32_t ble_sync_current_physical_page;
extern volatile uint32_t ble_sync_current_retry_count;

extern volatile uint16_t ble_sync_metadata_block_a;
extern volatile uint16_t ble_sync_metadata_block_b;
extern volatile uint32_t ble_sync_metadata_records_recovered;
extern volatile uint32_t ble_sync_metadata_records_written;
extern volatile uint32_t ble_sync_metadata_crc_errors;
extern volatile uint32_t ble_sync_metadata_write_errors;
extern volatile uint32_t ble_sync_metadata_erase_errors;

extern volatile uint32_t ble_sync_last_error;
extern volatile uint32_t ble_sync_last_abort_reason;
extern volatile uint32_t ble_sync_session_start_ms;
extern volatile uint32_t ble_sync_session_duration_ms;
extern volatile uint32_t ble_sync_diag_last_ack_ms;
extern volatile uint32_t ble_sync_diag_last_timeout_ms;
extern volatile uint32_t ble_sync_diag_last_timeout_kind;
extern volatile uint32_t ble_sync_diag_last_abort_ms;
extern volatile uint32_t ble_sync_diag_cleanup_ms;
extern volatile uint32_t ble_sync_diag_cleanup_count;
extern volatile uint32_t ble_sync_protocol_self_test_failures;
extern volatile BleSyncLatestDiagnostics ble_sync_latest;

int BleSync_Init(NandLogger *logger);
int BleSync_StartNewLogGeneration(void);
int BleSync_FactoryReset(NandLogger *logger);
int BleSync_StartSession(NandLogger *logger, uint32_t now_ms);
void BleSync_Process(NandLogger *logger, uint32_t now_ms, uint8_t usb_active);
void BleSync_RequestUsbPreemption(void);

#endif /* INC_BLE_SYNC_H_ */

/*
 * Memory_operations.h
 *
 *  Created on: Mar 27, 2024
 *      Author: alice
 */

#ifndef INC_MEMORY_OPERATIONS_H_
#define INC_MEMORY_OPERATIONS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "main.h"
#include "SPI.h"
#include "SPI_NAND.h"

#include "led_driver.h"

/*
 * Packet layout per sample (STRIDE_BYTES_PER_SAMPLE = 40, effective BYTES_PER_SAMPLE = 40):
 *   [0]      hh  (timestamp hours)
 *   [1]      mm  (timestamp minutes)
 *   [2]      ss  (timestamp seconds)
 *   [3..4]   sss (milliseconds, little-endian uint16)
 *   [5..10]  accelerometer XYZ (6 bytes raw, LSB first per axis)
 *   [11..16] gyroscope     XYZ (6 bytes raw, LSB first per axis)
 *   [17..38] legacy light area, now reserved and zero-filled
 *   [39]     reserved for future use / alignment
 */
#define BYTES_PER_SAMPLE         40U   /* logical sample payload size */
#define STRIDE_BYTES_PER_SAMPLE  40U   /* byte stride used in NAND_packet */
#define SAMPLES_PER_PAGE         (4096U / STRIDE_BYTES_PER_SAMPLE)

typedef struct bookmark
{
  uint16_t blocco_scritto;
  uint8_t pagina_scritta;
  int b;

}NAND_info;

typedef struct Time
{
  uint8_t hh;
  uint8_t mm;
  uint8_t ss;
  uint16_t sss;
} Time_Struct;

void find_bad_blocks(uint16_t *bad_blocks);
void erase_good_blocks(uint8_t *bad_blocks);
NAND_info read_memory(int b, NAND_info indice, uint16_t *blocco_letto, uint8_t *pagina_letta, uint16_t bad_blocks[2048], uint8_t *data_letto);
void write_info(NAND_info segnalibro, uint16_t bad_blocks[2048]);
NAND_info read_info(uint16_t bad_blocks[2048]);

void write_packet(uint16_t sample, Time_Struct timestamp,
                  uint8_t *accelerometer, uint8_t *gyroscope,
                  uint8_t *light_raw,
                  uint8_t *NAND_packet);

void erase_memory(void);
void write_memory(void);
void read_memory_and_transmit(void);
void write_audio_page(int16_t *audio_buffer, uint32_t audio_samples);


typedef enum
{
    LOG_OK = 0,
    LOG_ERR_FULL,
    LOG_ERR_NO_GOOD_BLOCKS,
    LOG_ERR_BAD_ARGUMENT,
    LOG_ERR_NAND,
    LOG_ERR_USB
} LogStatus;


#define NAND_TOTAL_BLOCKS        2048U
#define NAND_PAGES_PER_BLOCK     64U
#define NAND_PAGE_SIZE_BYTES     4096U

#define LOG_HEADER_SIZE_BYTES    16U
#define LOG_SENSOR_PAYLOAD_BYTES (NAND_PAGE_SIZE_BYTES - LOG_HEADER_SIZE_BYTES)
#define LOG_SENSOR_RECORD_BYTES  40U
#define LOG_SENSOR_RECORDS_PER_PAGE (LOG_SENSOR_PAYLOAD_BYTES / LOG_SENSOR_RECORD_BYTES)
#define LOG_LIGHT_RAW_RECORD_BYTES 28U
#define LOG_LIGHT_RAW_RECORDS_PER_PAGE (LOG_SENSOR_PAYLOAD_BYTES / LOG_LIGHT_RAW_RECORD_BYTES)
#define LOG_LIGHT_RAW_MAX_PAYLOAD_BYTES (LOG_LIGHT_RAW_RECORDS_PER_PAGE * LOG_LIGHT_RAW_RECORD_BYTES)
#define LOG_LIGHT_RAW_PADDING_BYTES (LOG_SENSOR_PAYLOAD_BYTES - LOG_LIGHT_RAW_MAX_PAYLOAD_BYTES)
#define LOG_AUDIO_FEATURE_RECORD_BYTES 24U
#define LOG_AUDIO_FEATURE_RECORDS_PER_PAGE \
    (LOG_SENSOR_PAYLOAD_BYTES / LOG_AUDIO_FEATURE_RECORD_BYTES)
#define LOG_AUDIO_FEATURE_MAX_PAYLOAD_BYTES \
    (LOG_AUDIO_FEATURE_RECORDS_PER_PAGE * LOG_AUDIO_FEATURE_RECORD_BYTES)

#define LOG_MAGIC_SENSOR 0x534E4553UL  /* 'SENS' */
#define LOG_MAGIC_AUDIO  0x30445541UL  /* 'AUD0' */
#define LOG_MAGIC_LIGHT_RAW 0x5741524CUL  /* 'LRAW' */
#define LOG_MAGIC_AUDIO_FEATURE 0x41454641UL  /* 'AFEA' */

typedef struct __attribute__((packed))
{
    uint32_t magic;
    uint8_t  version;
    uint8_t  header_size;
    uint16_t payload_bytes;
    uint32_t page_sequence;
    uint32_t timestamp_ms;
} LogPageHeader;

typedef struct __attribute__((packed))
{
    uint32_t sample_elapsed_ms;
    uint32_t sample_index;
    uint16_t f1_counts;
    uint16_t f2_counts;
    uint16_t f3_counts;
    uint16_t f4_counts;
    uint16_t f5_counts;
    uint16_t f6_counts;
    uint16_t f7_counts;
    uint16_t f8_counts;
    uint16_t clear_counts;
    uint16_t nir_counts;
} LightRawSampleRecord;

_Static_assert(sizeof(LightRawSampleRecord) == LOG_LIGHT_RAW_RECORD_BYTES,
               "Unexpected LightRawSampleRecord size");

typedef struct __attribute__((packed))
{
    uint32_t window_sequence;
    uint32_t window_start_ms;
    uint16_t sample_count;
    int16_t mean_counts_rounded;
    int16_t rms_z_centi_dbfs;
    int16_t rms_a_centi_dbfs;
    int16_t estimated_laeq_centi_dba;
    int16_t peak_centi_dbfs;
    uint16_t clipped_sample_count;
    uint8_t environment_class;
    uint8_t flags;
} AudioFeatureRecordV1;

_Static_assert(sizeof(AudioFeatureRecordV1) == LOG_AUDIO_FEATURE_RECORD_BYTES,
               "Unexpected AudioFeatureRecordV1 size");
_Static_assert(offsetof(AudioFeatureRecordV1, window_sequence) == 0U,
               "Unexpected AFEA window_sequence offset");
_Static_assert(offsetof(AudioFeatureRecordV1, window_start_ms) == 4U,
               "Unexpected AFEA window_start_ms offset");
_Static_assert(offsetof(AudioFeatureRecordV1, sample_count) == 8U,
               "Unexpected AFEA sample_count offset");
_Static_assert(offsetof(AudioFeatureRecordV1, mean_counts_rounded) == 10U,
               "Unexpected AFEA mean_counts_rounded offset");
_Static_assert(offsetof(AudioFeatureRecordV1, rms_z_centi_dbfs) == 12U,
               "Unexpected AFEA rms_z_centi_dbfs offset");
_Static_assert(offsetof(AudioFeatureRecordV1, rms_a_centi_dbfs) == 14U,
               "Unexpected AFEA rms_a_centi_dbfs offset");
_Static_assert(offsetof(AudioFeatureRecordV1, estimated_laeq_centi_dba) == 16U,
               "Unexpected AFEA estimated_laeq_centi_dba offset");
_Static_assert(offsetof(AudioFeatureRecordV1, peak_centi_dbfs) == 18U,
               "Unexpected AFEA peak_centi_dbfs offset");
_Static_assert(offsetof(AudioFeatureRecordV1, clipped_sample_count) == 20U,
               "Unexpected AFEA clipped_sample_count offset");
_Static_assert(offsetof(AudioFeatureRecordV1, environment_class) == 22U,
               "Unexpected AFEA environment_class offset");
_Static_assert(offsetof(AudioFeatureRecordV1, flags) == 23U,
               "Unexpected AFEA flags offset");
_Static_assert(LOG_AUDIO_FEATURE_RECORDS_PER_PAGE == 170U,
               "Unexpected AFEA page capacity");

typedef struct
{
    uint16_t good_blocks[NAND_TOTAL_BLOCKS];
    uint16_t good_block_count;

    uint16_t current_good_block_index;
    uint8_t  current_page_in_block;

    uint32_t page_sequence;

    uint8_t sensor_page_buffer[NAND_PAGE_SIZE_BYTES];
    uint16_t sensor_records_in_page;

    uint8_t light_raw_page_buffer[NAND_PAGE_SIZE_BYTES];
    uint16_t light_raw_records_in_page;
    uint16_t light_raw_payload_bytes;

    uint8_t audio_feature_page_buffer[NAND_PAGE_SIZE_BYTES];
    uint16_t audio_feature_records_in_page;
    uint16_t audio_feature_payload_bytes;
    uint32_t audio_feature_first_timestamp_ms;

    uint32_t light_pages_written;
    uint32_t light_partial_pages_flushed;
    uint32_t light_full_pages_flushed;
    uint32_t light_nand_write_failures;
    uint32_t light_nand_verify_failures;
    uint32_t light_payload_consistency_failures;
} NandLogger;

extern volatile uint32_t nand_erase_attempts;
extern volatile uint32_t nand_erase_failures;
extern volatile uint16_t nand_first_failed_erase_block;
extern volatile int32_t nand_last_erase_status;
extern volatile uint32_t light_prewrite_check_count;
extern volatile uint32_t light_prewrite_declared_payload_bytes;
extern volatile uint32_t light_prewrite_declared_record_count;
extern volatile uint32_t light_prewrite_first_all_ff_record;
extern volatile uint32_t light_prewrite_first_partial_record;
extern volatile uint32_t light_prewrite_last_non_ff_offset;
extern volatile uint32_t light_prewrite_current_record_count;
extern volatile uint32_t usb_tx_submit_count;
extern volatile uint32_t usb_tx_complete_count;
extern volatile uint32_t usb_tx_busy_retry_count;
extern volatile uint32_t usb_tx_fail_count;
extern volatile uint32_t usb_tx_timeout_count;
extern volatile uint16_t usb_tx_last_length;
extern volatile uint8_t usb_tx_last_status;
extern volatile uint32_t audio_feature_records_generated;
extern volatile uint32_t audio_feature_records_buffered;
extern volatile uint32_t audio_feature_records_persisted;
extern volatile uint32_t audio_feature_page_flush_count;
extern volatile uint32_t audio_feature_page_flush_errors;

LogStatus NANDLogger_Init(NandLogger *logger);

LogStatus NANDLogger_EraseAllGoodBlocks(NandLogger *logger);

LogStatus NANDLogger_AppendSensorRecord(NandLogger *logger,
                                        Time_Struct timestamp,
                                        const uint8_t *accelerometer,
                                        const uint8_t *gyroscope,
                                        const uint8_t *light_raw);

LogStatus NANDLogger_AppendAudioBuffer(NandLogger *logger,
                                       const int16_t *audio_buffer,
                                       uint32_t audio_samples,
                                       uint32_t timestamp_ms);

LogStatus NANDLogger_AppendAudioFeatureRecord(NandLogger *logger,
                                              const AudioFeatureRecordV1 *record);

LogStatus NANDLogger_AppendLightRawRecord(NandLogger *logger,
                                          const LightRawSampleRecord *record,
                                          uint32_t timestamp_ms);

LogStatus NANDLogger_DownloadAll(NandLogger *logger);
LogStatus NANDLogger_Flush(NandLogger *logger, uint32_t timestamp_ms);
LogStatus NANDLogger_FlushLightRaw(NandLogger *logger, uint32_t timestamp_ms);
LogStatus NANDLogger_FlushAudioFeatures(NandLogger *logger);
LogStatus NANDLogger_FlushWindowData(NandLogger *logger, uint32_t timestamp_ms);
LogStatus NANDLogger_FlushAll(NandLogger *logger, uint32_t timestamp_ms);

void NANDLogger_SerializeLightRawRecordForTest(uint8_t *dst,
                                               const LightRawSampleRecord *record);


#endif /* INC_MEMORY_OPERATIONS_H_ */

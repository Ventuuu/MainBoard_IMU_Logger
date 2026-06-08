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

/*
 * Packet layout per sample (STRIDE_BYTES_PER_SAMPLE = 40, effective BYTES_PER_SAMPLE = 40):
 *   [0]      hh  (timestamp hours)
 *   [1]      mm  (timestamp minutes)
 *   [2]      ss  (timestamp seconds)
 *   [3..4]   sss (milliseconds, little-endian uint16)
 *   [5..10]  accelerometer XYZ (6 bytes raw, LSB first per axis)
 *   [11..16] gyroscope     XYZ (6 bytes raw, LSB first per axis)
 *   [17..32] light spectral filters F1..F8 (8 channels × 2 bytes each, uint16
 *             little-endian, mapping defined in as7341_driver.c and host-side
 *             parser)
 *   [33..34] light Clear channel (2 bytes, LSB first uint16)
 *   [35..36] light NIR   channel (2 bytes, LSB first uint16)
 *   [37..38] mains flicker category in Hz (uint16: 0, 50 or 60)
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
    LOG_ERR_NAND
} LogStatus;


#define NAND_TOTAL_BLOCKS        2048U
#define NAND_PAGES_PER_BLOCK     64U
#define NAND_PAGE_SIZE_BYTES     4096U

#define LOG_HEADER_SIZE_BYTES    16U
#define LOG_SENSOR_PAYLOAD_BYTES (NAND_PAGE_SIZE_BYTES - LOG_HEADER_SIZE_BYTES)
#define LOG_SENSOR_RECORD_BYTES  40U
#define LOG_SENSOR_RECORDS_PER_PAGE (LOG_SENSOR_PAYLOAD_BYTES / LOG_SENSOR_RECORD_BYTES)

#define LOG_MAGIC_SENSOR 0x534E4553UL  /* 'SENS' */
#define LOG_MAGIC_AUDIO  0x30445541UL  /* 'AUD0' */

typedef enum
{
    LOG_OK = 0,
    LOG_ERR_FULL,
    LOG_ERR_NO_GOOD_BLOCKS,
    LOG_ERR_BAD_ARGUMENT,
    LOG_ERR_NAND
} LogStatus;


#define NAND_TOTAL_BLOCKS        2048U
#define NAND_PAGES_PER_BLOCK     64U
#define NAND_PAGE_SIZE_BYTES     4096U

#define LOG_HEADER_SIZE_BYTES    16U
#define LOG_SENSOR_PAYLOAD_BYTES (NAND_PAGE_SIZE_BYTES - LOG_HEADER_SIZE_BYTES)
#define LOG_SENSOR_RECORD_BYTES  40U
#define LOG_SENSOR_RECORDS_PER_PAGE (LOG_SENSOR_PAYLOAD_BYTES / LOG_SENSOR_RECORD_BYTES)

#define LOG_MAGIC_SENSOR 0x534E4553UL  /* 'SENS' */
#define LOG_MAGIC_AUDIO  0x30445541UL  /* 'AUD0' */


typedef struct __attribute__((packed))
{
    uint32_t magic;
    uint8_t  version;
    uint8_t  header_size;
    uint16_t payload_bytes;
    uint32_t page_sequence;
    uint32_t timestamp_ms;
} LogPageHeader;


typedef struct
{
    uint16_t good_blocks[NAND_TOTAL_BLOCKS];
    uint16_t good_block_count;

    uint16_t current_good_block_index;
    uint8_t  current_page_in_block;

    uint32_t page_sequence;

    uint8_t sensor_page_buffer[NAND_PAGE_SIZE_BYTES];
    uint16_t sensor_records_in_page;
} NandLogger;


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

LogStatus NANDLogger_DownloadAll(NandLogger *logger);
LogStatus NANDLogger_Flush(NandLogger *logger, uint32_t timestamp_ms);


#endif /* INC_MEMORY_OPERATIONS_H_ */

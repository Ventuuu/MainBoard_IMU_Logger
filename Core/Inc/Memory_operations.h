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

#endif /* INC_MEMORY_OPERATIONS_H_ */

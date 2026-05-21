/*
 * Memory_operations.c
 *
 *  This file contains the high-level operations that can be used for managing the SPI NAND
 *  You will need the SPI_NAND.c library.
 *
 */

#include "string.h"
#include "stdio.h"
#include "stdbool.h"
#include "main.h"
#include "SPI.h"
#include "SPI_NAND.h"
#include "Memory_operations.h"

NAND_info data;

/*
 * All'inizio faccio un spi_nand_init(); che non cancella il contenuto della memoria
 * Il flusso di operazioni sarà: inizializzo, leggo tutto, identifico i bad blocks, erase tutto, scrivo tutto
 */

void find_bad_blocks(uint16_t *bad_blocks){
	read_address_t blocco;
	blocco.block=0;
	blocco.page=0;
	blocco.dummy=0;
	bool is_bad_mark=true;
	int j = 0;
	for(int i = 0; i<2048; i++){
		blocco.block=i;
		spi_nand_block_is_bad(blocco, &is_bad_mark);
		if(!is_bad_mark) {
		  bad_blocks[j]=i;
		  j++;
		}
	}
}

void erase_good_blocks(uint8_t *bad_blocks){
	read_address_t blocco;
	blocco.block=0;
	blocco.page=0;
	blocco.dummy=0;
	bool is_bad_mark=true;
	for(int i = 0; i<2048; i++){
		blocco.block=i;
		spi_nand_block_is_bad(blocco, &is_bad_mark);
		if(is_bad_mark){
		  bad_blocks[i]=1;
		}
		if(!is_bad_mark) {
		  bad_blocks[i]=0;
		  spi_nand_block_erase(blocco);
		}
	}
}

/**
 * @brief Assembles one data record into the NAND page buffer.
 *
 * Packet layout (40 bytes per sample):
 *   [0]      hh
 *   [1]      mm
 *   [2]      ss
 *   [3..4]   sss  (uint16, little-endian)
 *   [5..10]  accelerometer raw bytes (XL, XH, YL, YH, ZL, ZH)
 *   [11..16] gyroscope     raw bytes (XL, XH, YL, YH, ZL, ZH)
 *   [17..32] light spectral filters F1..F8 (8 channels × 2 bytes each,
 *             little-endian uint16)
 *   [33..34] light Clear   (uint16, little-endian)
 *   [35..36] light NIR     (uint16, little-endian)
 *   [37..38] mains flicker category in Hz (uint16: 0, 50 or 60)
 *   [39]     reserved / currently zeroed.
 */
void write_packet(uint16_t sample, Time_Struct timestamp,
                  uint8_t *accelerometer, uint8_t *gyroscope,
                  uint8_t *light_raw,
                  uint8_t *NAND_packet)
{
	uint16_t base = sample * STRIDE_BYTES_PER_SAMPLE;

	/* Timestamp */
	NAND_packet[base + 0] = timestamp.hh;
	NAND_packet[base + 1] = timestamp.mm;
	NAND_packet[base + 2] = timestamp.ss;

	uint16_t milli = timestamp.sss;
	NAND_packet[base + 3] = (uint8_t)(milli & 0xFF);
	NAND_packet[base + 4] = (uint8_t)(milli >> 8);

	/* Accelerometer (6 bytes) */
	NAND_packet[base + 5]  = accelerometer[0];
	NAND_packet[base + 6]  = accelerometer[1];
	NAND_packet[base + 7]  = accelerometer[2];
	NAND_packet[base + 8]  = accelerometer[3];
	NAND_packet[base + 9]  = accelerometer[4];
	NAND_packet[base + 10] = accelerometer[5];

	/* Gyroscope (6 bytes) */
	NAND_packet[base + 11] = gyroscope[0];
	NAND_packet[base + 12] = gyroscope[1];
	NAND_packet[base + 13] = gyroscope[2];
	NAND_packet[base + 14] = gyroscope[3];
	NAND_packet[base + 15] = gyroscope[4];
	NAND_packet[base + 16] = gyroscope[5];

	/* Light filters F1..F8 (16 bytes: 8 channels × 2 bytes, indices 0..15) */
	for (uint8_t i = 0; i < 16; i++) {
		NAND_packet[base + 17 + i] = light_raw[i];
	}

	/* Clear and NIR (4 bytes: indices 16..19) */
	NAND_packet[base + 33] = light_raw[16];  /* Clear LSB */
	NAND_packet[base + 34] = light_raw[17];  /* Clear MSB */
	NAND_packet[base + 35] = light_raw[18];  /* NIR LSB   */
	NAND_packet[base + 36] = light_raw[19];  /* NIR MSB   */

	/* Mains flicker category (2 bytes: indices 20..21, little-endian) */
	NAND_packet[base + 37] = light_raw[20];
	NAND_packet[base + 38] = light_raw[21];

	/* Reserved byte: zero for now */
	NAND_packet[base + 39] = 0x00;
}

/*
 * Memory_operations.c
 *
 * High-level operations for managing SPI NAND logging.
 *
 * New architecture:
 * - One sequential NAND logger for all data types.
 * - Sensor pages are marked with LOG_MAGIC_SENSOR = 'SENS'.
 * - Audio pages are marked with LOG_MAGIC_AUDIO  = 'AUD0'.
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
 *
 * Audio page payload:
 *  - int16_t PCM samples
 *  - AUDIO_BUFFER_SIZE = 1024 samples -> 2048 bytes
 */

#include "string.h"
#include "stdio.h"
#include "stdbool.h"
#include "main.h"
#include "SPI.h"
#include "SPI_NAND.h"
#include "Memory_operations.h"
#include "usbd_cdc_if.h"



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

    if (logger->current_good_block_index >= logger->good_block_count)
    {
        return LOG_ERR_FULL;
    }

    return LOG_OK;
}


static LogStatus logger_write_current_page(NandLogger *logger,
                                           const uint8_t *page)
{
    read_address_t addr;
    column_address_t column = 0;

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
    spi_nand_page_program(addr,
                          column,
                          (uint8_t *)page,
                          NAND_PAGE_SIZE_BYTES);

    logger->page_sequence++;

    return logger_advance_page(logger);
}


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

    memset(logger->sensor_page_buffer, 0xFF, NAND_PAGE_SIZE_BYTES);

    return LOG_OK;
}


LogStatus NANDLogger_EraseAllGoodBlocks(NandLogger *logger)
{
    read_address_t addr;

    if (logger == NULL)
    {
        return LOG_ERR_BAD_ARGUMENT;
    }

    if (logger->good_block_count == 0U)
    {
        return LOG_ERR_NO_GOOD_BLOCKS;
    }

    addr.page = 0U;
    addr.dummy = 0U;

    for (uint16_t i = 0U; i < logger->good_block_count; i++)
    {
        addr.block = logger->good_blocks[i];
        spi_nand_block_erase(addr);
    }

    logger->current_good_block_index = 0U;
    logger->current_page_in_block = 0U;
    logger->page_sequence = 0U;
    logger->sensor_records_in_page = 0U;

    memset(logger->sensor_page_buffer, 0xFF, NAND_PAGE_SIZE_BYTES);

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

LogStatus NANDLogger_Flush(NandLogger *logger, uint32_t timestamp_ms)
{
    return logger_flush_sensor_page(logger, timestamp_ms);
}

static LogStatus logger_usb_send(const uint8_t *data, uint16_t len)
{
    uint32_t start_tick;

    if (data == NULL)
    {
        return LOG_ERR_BAD_ARGUMENT;
    }

    start_tick = HAL_GetTick();

    while (CDC_Transmit_FS((uint8_t *)data, len) == USBD_BUSY)
    {
        if ((HAL_GetTick() - start_tick) > 1000U)
        {
            return LOG_ERR_NAND;
        }
    }

    /*
     * Delay breve per non saturare la USB CDC.
     * Dopo i primi test puoi ridurlo o rimuoverlo.
     */
    HAL_Delay(1U);

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
    status = NANDLogger_Flush(logger, HAL_GetTick());
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
     * Invio marker iniziale: 8 byte = "LOGSTART".
     * Il PC userà questo per sincronizzarsi.
     */
    status = logger_usb_send(start_marker, sizeof(start_marker));
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

        if (status != LOG_OK)
        {
            return status;
        }
    }

    /*
     * Marker finale: 8 byte = "LOGEND!!".
     */
    status = logger_usb_send(end_marker, sizeof(end_marker));
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
 * [17..32] light F1..F8, 16 bytes
 * [33..34] Clear
 * [35..36] NIR
 * [37..38] mains flicker
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

    /* Light spectral filters F1..F8, 16 bytes */
    for (uint8_t i = 0U; i < 16U; i++)
    {
        packet_buffer[base + 17U + i] = light_raw[i];
    }

    /* Clear and NIR */
    packet_buffer[base + 33U] = light_raw[16];  /* Clear LSB */
    packet_buffer[base + 34U] = light_raw[17];  /* Clear MSB */
    packet_buffer[base + 35U] = light_raw[18];  /* NIR LSB   */
    packet_buffer[base + 36U] = light_raw[19];  /* NIR MSB   */

    /* Mains flicker */
    packet_buffer[base + 37U] = light_raw[20];
    packet_buffer[base + 38U] = light_raw[21];

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
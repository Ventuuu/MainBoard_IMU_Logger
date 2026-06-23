# AS7341 Raw Counts Pipeline

## 1. Scopo della pipeline

La pipeline luce attiva salva nella NAND la serie temporale dei counts ADC grezzi del sensore AS7341 durante la sessione di acquisizione.

Per ogni campione completo vengono salvati:

- F1, F2, F3, F4, F5, F6, F7, F8;
- Clear;
- NIR;
- timestamp relativo all'inizio sessione;
- indice progressivo del campione luce.

Non vengono applicati normalizzazione, media finale di sessione, dark correction, classificazione luminosa, lux, irradianza, smoothing o clamp artificiale. La vecchia pipeline `LITE` non viene piu' chiamata dal percorso attivo.

## 2. Flusso di acquisizione

La prima pressione dello USER BUTTON imposta `start_acquisition_requested` nella ISR e il main loop avvia la sessione da `STATE_IDLE`.

Durante lo start:

- viene cancellata la NAND tramite `NANDLogger_EraseAllGoodBlocks()`;
- timestamp, contatori e buffer luce vengono azzerati;
- `light_session_start_ms = HAL_GetTick()`;
- `light_sample_index = 0`;
- parte TIM2 con interrupt;
- lo stato diventa `STATE_ACQUISITION` e il LED verde resta acceso.

Durante `STATE_ACQUISITION`, l'interrupt TIM2 incrementa solo `sensor_tick_pending`. Il main loop consuma i tick, legge IMU, invia BLE, salva il record `SENS` e ogni 8 tick acquisisce un campione AS7341 completo e lo accoda alla pagina `LRAW`.

La seconda pressione dello USER BUTTON imposta `stop_acquisition_requested`. Il main loop ferma TIM2, ferma il microfono se attivo, porta subito lo stato a `STATE_IDLE`, spegne il LED verde e poi chiama `NANDLogger_FlushAll()`. Il flush finale scrive sia la pagina `SENS` parziale sia la pagina `LRAW` parziale, se non vuote.

## 3. Frequenza di campionamento

TIM2 e' lo scheduler principale:

- prescaler: `7200 - 1`;
- periodo: `99`;
- tick applicativo: 100 Hz, cioe' 10 ms.

La luce usa `LIGHT_SUBSAMPLE_TICKS = 8`, quindi viene richiesta una lettura AS7341 completa ogni 8 tick:

- periodo effettivo minimo: circa 80 ms tra due campioni completi;
- frequenza effettiva nominale: 12.5 Hz.

Un campione completo richiede due fasi SMUX. Con la configurazione AS7341 attuale, il tempo di integrazione per fase e' circa 27.8 ms; due fasi richiedono circa 55.6 ms piu' overhead I2C/SMUX.

## 4. Configurazione AS7341

La configurazione effettivamente applicata dopo `AS7341_Init()` e':

- gain: `AS7341_PROCESSING_GAIN = AS7341_GAIN_64X`;
- `ATIME = AS7341_PROCESSING_ATIME = 9`;
- `ASTEP = AS7341_PROCESSING_ASTEP = 999`;
- tempo integrazione per fase: `(ATIME + 1) * (ASTEP + 1) * 2.78 us = 27.8 ms`;
- tempo per campione completo: due integrazioni, circa 55.6 ms;
- fondo scala salvabile nel record: `uint16_t`, quindi 0..65535 counts;
- auto-zero: non configurato dal firmware;
- AGC: non configurato dal firmware.

Il codice scrive i registri `ATIME`, `ASTEP_L`, `ASTEP_H` e `AGAIN` tramite `AS7341_ConfigTimingAndGain()`.

## 5. Mappatura SMUX

La lettura completa in `AS7341_ReadFullSpectrum()` usa due configurazioni SMUX.

Prima fase:

| ADC interno | Canale fisico |
| --- | --- |
| CH0 | F1 |
| CH1 | F2 |
| CH2 | F3 |
| CH3 | F4 |
| CH4 | Clear fase 1 |
| CH5 | NIR fase 1 |

Seconda fase:

| ADC interno | Canale fisico |
| --- | --- |
| CH0 | F5 |
| CH1 | F6 |
| CH2 | F7 |
| CH3 | F8 |
| CH4 | Clear fase 2 |
| CH5 | NIR fase 2 |

Un record viene salvato solo se entrambe le fasi SMUX e le due letture dei sei ADC terminano correttamente. In caso di errore I2C/SMUX, il campione viene scartato, `sample_index` non aumenta e `light_samples_discarded` viene incrementato.

Clear e NIR vengono salvati come singolo valore mediando le due fasi con somma a 32 bit:

```c
clear_counts = (uint16_t)(((uint32_t)clear_phase_1 + (uint32_t)clear_phase_2 + 1U) / 2U);
nir_counts   = (uint16_t)(((uint32_t)nir_phase_1   + (uint32_t)nir_phase_2   + 1U) / 2U);
```

## 6. Formato del record

Tipo logico: `LightRawSampleRecord`.

| Offset | Campo | Tipo | Size |
| ---: | --- | --- | ---: |
| 0 | `sample_elapsed_ms` | `uint32_t` | 4 |
| 4 | `sample_index` | `uint32_t` | 4 |
| 8 | `f1_counts` | `uint16_t` | 2 |
| 10 | `f2_counts` | `uint16_t` | 2 |
| 12 | `f3_counts` | `uint16_t` | 2 |
| 14 | `f4_counts` | `uint16_t` | 2 |
| 16 | `f5_counts` | `uint16_t` | 2 |
| 18 | `f6_counts` | `uint16_t` | 2 |
| 20 | `f7_counts` | `uint16_t` | 2 |
| 22 | `f8_counts` | `uint16_t` | 2 |
| 24 | `clear_counts` | `uint16_t` | 2 |
| 26 | `nir_counts` | `uint16_t` | 2 |

Dimensione totale: 28 byte.

Endianness: little-endian esplicito, campo per campo, tramite helper `logger_put_u16_le()` e `logger_put_u32_le()`. La compatibilita' Python e':

```python
"<II10H"
```

Record di test:

```text
elapsed=1234, index=7, F1..NIR=100,200,300,400,500,600,700,800,900,1000
```

Byte attesi:

```text
D2 04 00 00 07 00 00 00 64 00 C8 00 2C 01 90 01
F4 01 58 02 BC 02 20 03 84 03 E8 03
```

## 7. Formato NAND

Le pagine raw luce usano magic:

```c
#define LOG_MAGIC_LIGHT_RAW 0x5741524CUL /* 'LRAW' */
```

In memoria little-endian i primi 4 byte sono `4C 52 41 57`, cioe' `LRAW`.

Header pagina riutilizzato:

| Offset | Campo | Tipo | Size |
| ---: | --- | --- | ---: |
| 0 | `magic` | `uint32_t` | 4 |
| 4 | `version` | `uint8_t` | 1 |
| 5 | `header_size` | `uint8_t` | 1 |
| 6 | `payload_bytes` | `uint16_t` | 2 |
| 8 | `page_sequence` | `uint32_t` | 4 |
| 12 | `timestamp_ms` | `uint32_t` | 4 |

Costanti reali:

- page size: 4096 byte;
- header: 16 byte;
- payload massimo: 4080 byte;
- record `LRAW`: 28 byte;
- massimo record per pagina: `floor(4080 / 28) = 145`;
- payload pagina piena: `145 * 28 = 4060` byte;
- padding residuo pagina piena: 20 byte.

`payload_bytes` contiene sempre solo `numero_record_validi * 28`: non include header, padding o spazio non usato.

Quando la pagina `LRAW` e' piena, `logger_flush_light_raw_page()` prepara l'header, programma la pagina NAND, incrementa `light_pages_written` solo se la scrittura riesce e inizializza un nuovo buffer a `0xFF`.

Alla pagina parziale viene applicata la stessa logica. Non viene scritta una pagina vuota. Il padding resta `0xFF`.

## 8. Interazione con gli altri sensori

L'IMU resta attiva: `ProcessSensorTick()` continua a leggere accelerometro e giroscopio, inviare i pacchetti BLE e salvare record `SENS`.

Il microfono non viene modificato nella logica di acquisizione: resta gestito da MDF DMA, `audio_buffer_ready`, `microphone_active` e `NANDLogger_AppendAudioBuffer()`.

I buffer sono separati:

- `sensor_page_buffer` per pagine `SENS`;
- `light_raw_page_buffer` per pagine `LRAW`;
- `logger_audio_page_buffer` per pagine `AUD0`.

Il record `SENS` resta da 40 byte. L'area legacy luce `[17..38]` viene mantenuta zero-filled tramite `raw_light[22]`.

## 9. Download USB

Il framing USB generale resta invariato:

- marker iniziale `LOGSTART`;
- `uint32_t` little-endian con numero pagine;
- invio pagine NAND complete da 4096 byte;
- marker finale `LOGEND!!`.

Prima del conteggio pagine, `NANDLogger_DownloadAll()` chiama `NANDLogger_FlushAll()`, quindi include anche eventuali pagine `SENS` e `LRAW` parziali. La validazione degli header riconosce `SENS`, `AUD0` e `LRAW`.

## 10. File modificati

- `Core/Inc/Memory_operations.h`: aggiunto `LightRawSampleRecord`, magic `LRAW`, dimensioni record/pagina, buffer e contatori diagnostici nel logger, API append/flush raw.
- `Core/Src/Memory_operations.c`: aggiunta serializzazione little-endian raw, accumulo pagine `LRAW`, flush pagina piena/parziale, validazione header `LRAW`, inclusione nel download.
- `Core/Inc/as7341_driver.h`: documentato l'ordine pubblico F1-F8/Clear/NIR.
- `Core/Src/as7341_driver.c`: le funzioni SMUX ora propagano errori; `AS7341_ReadFullSpectrum()` salva F1-F8 e media Clear/NIR.
- `Core/Src/main.c`: disattivato il percorso `LightMetrics/LITE`; aggiunto campionamento raw durante `STATE_ACQUISITION`, contatori diagnostici, flush stop, USER BUTTON rising + debounce.
- `MainBoard_IMU_Logger.ioc`: USER BUTTON impostato a `GPIO_MODE_IT_RISING`.
- `AS7341_RAW_COUNTS_PIPELINE.md`: questo documento.
- `AS7341_LIGHT_PIPELINE_CHANGES.md`: aggiunta nota iniziale che marca il documento `LITE` come storico/superato.

## 11. Test eseguiti

- Dimensione record: `_Static_assert(sizeof(LightRawSampleRecord) == 28U)` compilato con successo.
- Serializzazione: verificata funzione `logger_serialize_light_raw_record()` campo per campo little-endian; il record noto produce i byte indicati nella sezione 6.
- Record consecutivi: offset calcolati come `16 + n * 28`, quindi payload offset 0, 28, 56 dopo l'header.
- Pagina piena: capacita' `LOG_LIGHT_RAW_RECORDS_PER_PAGE = 145`; al record 145 viene eseguito flush senza superare 4096 byte.
- Pagina parziale: `NANDLogger_FlushLightRaw()`/`NANDLogger_FlushAll()` non scrivono pagine vuote e scrivono `payload_bytes = record_validi * 28`.
- Errore I2C/SMUX: `AS7341_ReadFullSpectrum()` ritorna 0 se una fase fallisce; il record non viene scritto e `sample_index` non aumenta.
- Start/stop USER BUTTON: ISR solo flag, rising edge con debounce; start da `STATE_IDLE`, stop da `STATE_ACQUISITION`, ritorno immediato a `STATE_IDLE` prima del flush.
- Compatibilita' IMU/microfono/USB: build completata; le funzioni IMU, MDF audio e framing USB restano nel percorso attivo.

## 12. Risultato della compilazione

Comando eseguito:

```bash
cmake --build --preset Debug
```

Risultato: compilazione e link completati con successo.

Warning rimasti:

- `Core/Src/SPI_NAND.c:488`: `-Wdiscarded-qualifiers` su chiamata a `spi_write()`;
- `Core/Src/SPI_NAND.c:48`: variabile statica `current_state` definita ma non usata.

Questi warning erano in `SPI_NAND.c` e non dipendono dalla pipeline `LRAW`.

## 13. Assunzioni e problemi aperti

- Non e' stato possibile verificare fisicamente su hardware NAND/AS7341/USB da Codex.
- Il fondo scala effettivamente utile puo' saturare prima del limite `uint16_t` in base alla luce e al gain 64x.
- Auto-zero e AGC non risultano configurati dal firmware; una calibrazione futura puo' decidere se abilitarli.
- La vecchia implementazione `light_metrics_mcu.c/h` resta nel progetto e puo' ancora essere compilata, ma non e' piu' chiamata da `main.c`.
- Le pagine `LITE` precedenti eventualmente gia' presenti in NAND non vengono prodotte dalla nuova sessione, perche' lo start cancella i blocchi buoni.

## 14. Vincoli Git

Durante questa modifica non sono stati eseguiti:

- commit;
- push;
- creazione di branch;
- cambio di branch.

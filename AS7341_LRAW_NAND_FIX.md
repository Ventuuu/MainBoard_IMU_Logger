# AS7341 LRAW NAND Fix

## 1. Evidenza iniziale

Il parser host ha osservato:

```text
page=11, sequence=11, payload=4060, records=145,
valid_records=3, first_ff_record=3

page=24, sequence=24, payload=3948, records=141,
valid_records=2, first_ff_record=2
```

Quindi l'header dichiarava molti piu' record di quelli realmente diversi da `0xFF` nel payload NAND.

Una seconda analisi ha mostrato un taglio ancora piu' preciso:

```text
header NAND:          16 byte
record LRAW 0:        28 byte
record LRAW 1 valido: 18 byte
totale programmato:   62 byte
```

Nel secondo record i campi fino a F5 erano validi, mentre F6, F7, F8, Clear e NIR risultavano `0xFFFF`. Il record successivo era interamente `0xFF`.

## 2. Causa reale trovata

Il dump host mostrava valori `0xFFFFFFFF` e `0xFFFF`, tipici di padding NAND cancellato a `0xFF` interpretato come record validi.

Nel codice analizzato non era piu' presente una assegnazione diretta errata del tipo:

```c
payload_bytes = 4080;
payload_bytes = PAGE_SIZE - HEADER_SIZE;
payload_bytes = sizeof(page_buffer) - sizeof(header);
```

La prima fragilita' era che la pagina `LRAW` manteneva solo `light_raw_records_in_page` e ricavava `payload_bytes` al flush. Questo rendeva meno verificabile la differenza tra:

- offset assoluto nella pagina;
- capacita' massima del payload;
- byte validi effettivi.

La seconda causa concreta era in `Core/Src/SPI_NAND.c`: `spi_nand_page_program()` chiamava `program_load()`, ma ignorava il suo valore di ritorno e chiamava comunque `program_execute()`.

```c
ret = program_load(column, data_in, write_len, SPI_TIMEOUT);
//if (SPI_NAND_RET_OK != ret) return ret;

return program_execute(row, SPI_TIMEOUT);
```

Se `program_load()` falliva o caricava solo una parte della cache NAND, il firmware poteva comunque programmare la pagina e incrementare `page_sequence`, producendo esattamente una pagina con pochi record iniziali validi e il resto `0xFF`.

La nuova analisi SPI non ha trovato un `tx_buffer[64]` esplicito. Il trasferimento dati era pero' fatto con una singola chiamata:

```c
HAL_SPI_Transmit(&hspi2, data_in, write_len, SPI_TIMEOUT);
```

senza conteggio dei byte realmente trasferiti. L'evidenza host indicava 62 byte dati programmati; la fix evita qualunque limite implicito del livello HAL/SPI dividendo il payload in chunk da 32 byte e mantenendo CS basso per tutta la transazione Program Load.

La fix:

- rende esplicito `light_raw_payload_bytes`;
- valida i record reali nel buffer prima del flush;
- controlla il ritorno di `program_load()`;
- valida i record reali anche dopo readback NAND.
- trasmette Program Load in chunk sotto lo stesso CS;
- misura `requested_bytes` e `transmitted_bytes` prima di permettere `program_execute()`.

## 3. File modificati

- `Core/Inc/Memory_operations.h`
  - aggiunti `LOG_LIGHT_RAW_MAX_PAYLOAD_BYTES` e `LOG_LIGHT_RAW_PADDING_BYTES`;
  - aggiunto `light_raw_payload_bytes` nello stato del logger;
  - aggiunti contatori diagnostici per full flush, verify failure e consistency failure.
- `Core/Src/Memory_operations.c`
  - append `LRAW` basato su payload byte esplicito;
  - flush `LRAW` basato su `light_raw_payload_bytes`;
  - controllo consistenza `record_count * 28 == payload_bytes`;
  - validazione che ogni record dichiarato non sia completamente `0xFF`;
  - controllo ritorno reale di `spi_nand_page_program()`;
  - readback diagnostico `NAND_VERIFY_LRAW_AFTER_WRITE` esteso al contenuto dei record;
  - validazione header `LRAW` limitata a massimo 4060 byte.
- `Core/Src/SPI_NAND.c`
  - `spi_nand_page_program()` ora ritorna subito l'errore di `program_load()`.
  - `program_load()` mantiene CS basso e invia command, address e dati in una singola transazione logica.
  - il payload viene inviato in chunk da 32 byte tramite `spi_write_counted()`.
  - aggiunti contatori `nand_program_load_requested_bytes`, `nand_program_load_transmitted_bytes`, `nand_spi_chunk_count`.
  - aggiunto self-test disattivabile `NAND_PAGE_PROGRAM_SELF_TEST`.
- `AS7341_LRAW_NAND_FIX.md`
  - questo documento.

## 4. Buffer e contatori

Architettura trovata:

- `SENS`: `logger->sensor_page_buffer`;
- `LRAW`: `logger->light_raw_page_buffer`;
- `AUD0`: `logger_audio_page_buffer`;
- download/readback: `logger_download_page_buffer`.

Il buffer `LRAW` e' dedicato, non condiviso con `SENS` o `AUD0`.

Contatori per pagina `LRAW`:

- `light_raw_records_in_page`;
- `light_raw_payload_bytes`.

Contatore globale/sessione del campione luce:

- `light_sample_index` in `main.c`.

La fix separa esplicitamente i contatori per pagina dal contatore globale del campione.

## 5. Capacita' pagina

Costanti reali:

```text
PAGE_SIZE   = 4096
HEADER_SIZE = 16
RECORD_SIZE = 28
```

Payload fisico disponibile:

```text
4096 - 16 = 4080 byte
```

Record completi massimi:

```text
floor(4080 / 28) = 145 record
```

Payload massimo valido:

```text
145 * 28 = 4060 byte
```

Padding inutilizzato:

```text
4080 - 4060 = 20 byte
```

Quindi una pagina piena `LRAW` deve dichiarare `payload_bytes = 4060`, non `4080`.

## 6. Ordine append corretto

Durante append:

```c
record_page_offset = LOG_HEADER_SIZE_BYTES + light_raw_payload_bytes;
```

Dopo serializzazione riuscita:

```c
validate record non-FF
light_raw_payload_bytes += LOG_LIGHT_RAW_RECORD_BYTES;
light_raw_records_in_page++;
```

Prima della scrittura:

```c
header.payload_bytes = light_raw_payload_bytes;
```

Controllo di coerenza:

```c
light_raw_payload_bytes ==
light_raw_records_in_page * LOG_LIGHT_RAW_RECORD_BYTES
```

I contatori vengono incrementati solo dopo che i 28 byte sono stati serializzati e validati.

## 7. Validazione prima del flush

Prima di programmare una pagina `LRAW`, il firmware conta i record realmente validi:

```c
valid_records =
    logger_count_valid_light_raw_records(light_raw_page_buffer,
                                         light_raw_records_in_page,
                                         &first_bad_record);
```

La condizione obbligatoria e':

```c
valid_records == light_raw_records_in_page
```

Se una pagina dichiara 145 record ma dal record 3 in avanti e' `0xFF`, il flush fallisce prima della programmazione NAND e aggiorna:

```c
light_non_ff_records_before_flush = 3;
light_first_ff_record_before_flush = 3;
light_payload_consistency_failures++;
```

## 8. Pagina piena

Con 145 record:

```text
light_raw_records_in_page = 145
light_raw_payload_bytes   = 4060
header.payload_bytes      = 4060
```

Gli ultimi 20 byte della zona payload restano padding `0xFF`, ma non sono conteggiati nel payload.

Il record numero 146 viene scritto in una nuova pagina:

```text
prima pagina: 145 record, payload_bytes = 4060
seconda pagina: 1 record, payload_bytes = 28
```

## 9. Pagina parziale

Con 1 record:

```text
payload_bytes = 28
```

Con 2 record:

```text
payload_bytes = 56
```

Con 144 record:

```text
payload_bytes = 4032
```

Se `light_raw_records_in_page == 0`, il flush non scrive pagine `LRAW` vuote.

## 10. Reset del buffer

Dopo una scrittura `LRAW` riuscita:

```c
light_raw_records_in_page = 0;
light_raw_payload_bytes = 0;
memset(light_raw_page_buffer, 0xFF, NAND_PAGE_SIZE_BYTES);
```

I byte della pagina precedente non vengono conteggiati nella pagina successiva.

Se un reset del buffer viene richiesto mentre `light_raw_records_in_page > 0` o `light_raw_payload_bytes > 0`, viene incrementato:

```c
light_buffer_resets_while_nonempty
```

## 11. Verifica readback

E' stato aggiunto:

```c
#define NAND_VERIFY_LRAW_AFTER_WRITE 1U
```

Quando attivo, dopo ogni scrittura `LRAW` il firmware rilegge la pagina appena programmata e verifica:

- magic `LRAW`;
- version `1`;
- header size `16`;
- `payload_bytes` uguale al payload atteso;
- `page_sequence`;
- `payload_bytes % 28 == 0`;
- `payload_bytes <= 4060`;
- payload scritto uguale al buffer sorgente.
- tutti i record dichiarati nel payload non sono completamente `0xFF`;
- `sample_elapsed_ms != UINT32_MAX`;
- `sample_index != UINT32_MAX`.
- nessun record dichiarato valido ha una coda cancellata `0xFF` lunga almeno 8 byte.

In caso di mismatch incrementa:

```c
light_nand_verify_failures
```

e ritorna `LOG_ERR_NAND`.

Il primo record `0xFF` trovato dopo readback viene salvato in:

```c
light_first_ff_record_after_readback
```

Il primo record parziale viene salvato in:

```c
light_first_partial_record_after_readback
light_first_partial_byte_offset_after_readback
```

Per il problema osservato ci si aspetterebbe:

```text
light_first_partial_record_after_readback = 1
light_first_partial_byte_offset_after_readback = 18
```

Con la fix, questi valori devono restare `UINT32_MAX`.

## 11.1 Program Load SPI

La sequenza Program Load resta:

```text
CS low
CMD_PROGRAM_LOAD
2 byte column address
data payload in chunk da 32 byte
CS high
```

`program_execute()` viene chiamato solo se:

```c
nand_program_load_transmitted_bytes ==
nand_program_load_requested_bytes
```

Contatori disponibili:

```c
nand_program_load_requested_bytes
nand_program_load_transmitted_bytes
nand_program_load_calls
nand_program_load_partial_failures
nand_spi_chunk_count
nand_program_load_first_failed_chunk
```

Valori attesi:

| Richiesta | Trasmissione attesa | Chunk dati attesi |
| ---: | ---: | ---: |
| 62 | 62 | 2 |
| 63 | 63 | 2 |
| 64 | 64 | 2 |
| 65 | 65 | 3 |
| 128 | 128 | 4 |
| 4060 | 4060 | 127 |
| 4096 | 4096 | 128 |

## 12. Conteggio pagine USB

Il framing USB resta invariato:

```text
LOGSTART
uint32_t total_pages
total_pages * 4096 byte
LOGEND!!
```

`NANDLogger_DownloadAll()` chiama `NANDLogger_FlushAll()` prima di leggere `logger->page_sequence`, quindi include anche l'ultima pagina `LRAW` parziale.

`page_sequence` aumenta solo se `spi_nand_page_program()` ritorna `SPI_NAND_RET_OK`. Pagine non scritte o scritte con errore NAND non vengono conteggiate come pagine valide.

## 13. Diagnostica debugger

Contatori e valori osservabili:

```c
light_records_serialized
light_records_counter_incremented
light_records_appended
light_page_buffer_resets
light_buffer_resets_while_nonempty
light_non_ff_records_before_flush
light_first_ff_record_before_flush
light_first_ff_record_after_readback
light_page_type_switches
light_payload_consistency_failures
light_wrong_buffer_failures
light_serialization_buffer_address
light_programmed_buffer_address
```

`light_serialization_buffer_address` deve puntare dentro `logger->light_raw_page_buffer`; `light_programmed_buffer_address` deve essere l'indirizzo base dello stesso buffer.

## 14. Risultati dei test

Verifiche statiche e di build:

- `sizeof(LightRawSampleRecord) == 28U` tramite `_Static_assert`;
- `sizeof(LogPageHeader) == 16U` tramite `_Static_assert`;
- build firmware completata con successo.

Verifiche logiche sul codice:

| Caso | Record | Payload atteso | Esito |
| --- | ---: | ---: | --- |
| Nessun record | 0 | 0 | nessuna pagina `LRAW` scritta |
| Un record | 1 | 28 | header usa 28 |
| Due record | 2 | 56 | header usa 56 |
| Tre record | 3 | 84 | validazione pre-flush richiede 3 record non-FF |
| 144 record | 144 | 4032 | pagina non supera capacita' |
| 145 record | 145 | 4060 | pagina piena flushata |
| 146 record | 145 + 1 | 4060 + 28 | record 146 in nuova pagina |
| Caso 3/145 | 145 dichiarati, 3 non-FF | errore | intercettato prima del flush |
| Caso 2/141 | 141 dichiarati, 2 non-FF | errore | intercettato prima del flush |
| Record parziale a byte 18 | record 1 | errore | intercettato da readback/partial-tail |

Readback hardware: implementato nel firmware tramite `NAND_VERIFY_LRAW_AFTER_WRITE`; non verificabile fisicamente da Codex senza board collegata in esecuzione.

Test SPI implementato:

```c
spi_nand_page_program_self_test(read_address_t row)
```

Con `NAND_PAGE_PROGRAM_SELF_TEST = 1U`, crea un pattern da 4096 byte, programma una pagina, la rilegge e salva:

```c
nand_program_self_test_first_mismatch_offset
nand_program_self_test_expected_byte
nand_program_self_test_read_byte
nand_program_self_test_requested_bytes
nand_program_self_test_transmitted_bytes
```

Prima della fix il primo mismatch atteso era intorno all'offset 62. Dopo la fix il valore atteso e' `UINT32_MAX`, cioe' nessun mismatch.

## 15. Procedura test su board

1. Flashare il firmware.
2. Avviare una breve acquisizione con USER BUTTON.
3. Fermare con seconda pressione.
4. Prima del download, controllare da debugger:

```text
light_payload_consistency_failures == 0
light_nand_verify_failures == 0
light_first_ff_record_before_flush == UINT32_MAX
light_first_ff_record_after_readback == UINT32_MAX
```

5. Eseguire download USB.
6. Verificare sul parser host:

```text
records dichiarati da payload_bytes
=
records realmente non-FF nel payload
```

## 16. Risultato compilazione

Comando:

```bash
cmake --build --preset Debug
```

Risultato: compilazione e link completati con successo.

Warning rimasti:

- `Core/Src/SPI_NAND.c`: `current_state` definita ma non usata.

Il warning `-Wdiscarded-qualifiers` su `spi_write()` e' stato rimosso rendendo il parametro `const uint8_t *`.

## 17. Problemi aperti

- La conferma finale del readback richiede esecuzione su hardware reale.
- Il parser host deve comunque rispettare `payload_bytes` dell'header e non leggere sempre 4080 byte.
- La NAND viene inviata via USB a pagine complete da 4096 byte: questo e' corretto; la validita' dei dati interni e' definita solo da `payload_bytes`.

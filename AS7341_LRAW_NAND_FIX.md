# AS7341 LRAW NAND Fix

## 1. Causa reale trovata

Il dump host mostrava valori `0xFFFFFFFF` e `0xFFFF`, tipici di padding NAND cancellato a `0xFF` interpretato come record validi.

Nel codice analizzato non era piu' presente una assegnazione diretta errata del tipo:

```c
payload_bytes = 4080;
payload_bytes = PAGE_SIZE - HEADER_SIZE;
payload_bytes = sizeof(page_buffer) - sizeof(header);
```

La fragilita' reale era che la pagina `LRAW` manteneva solo `light_raw_records_in_page` e ricavava `payload_bytes` al flush. Questo rendeva meno verificabile la differenza tra:

- offset assoluto nella pagina;
- capacita' massima del payload;
- byte validi effettivi.

La fix rende esplicito `light_raw_payload_bytes` e usa questo valore come unica sorgente per `header.payload_bytes`.

## 2. File modificati

- `Core/Inc/Memory_operations.h`
  - aggiunti `LOG_LIGHT_RAW_MAX_PAYLOAD_BYTES` e `LOG_LIGHT_RAW_PADDING_BYTES`;
  - aggiunto `light_raw_payload_bytes` nello stato del logger;
  - aggiunti contatori diagnostici per full flush, verify failure e consistency failure.
- `Core/Src/Memory_operations.c`
  - append `LRAW` basato su payload byte esplicito;
  - flush `LRAW` basato su `light_raw_payload_bytes`;
  - controllo consistenza `record_count * 28 == payload_bytes`;
  - controllo ritorno reale di `spi_nand_page_program()`;
  - readback diagnostico `NAND_VERIFY_LRAW_AFTER_WRITE`;
  - validazione header `LRAW` limitata a massimo 4060 byte.
- `AS7341_LRAW_NAND_FIX.md`
  - questo documento.

## 3. Capacita' pagina

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

## 4. Gestione payload_bytes

Durante append:

```c
record_page_offset = LOG_HEADER_SIZE_BYTES + light_raw_payload_bytes;
```

Dopo serializzazione riuscita:

```c
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

## 5. Pagina piena

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

## 6. Pagina parziale

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

## 7. Reset del buffer

Dopo una scrittura `LRAW` riuscita:

```c
light_raw_records_in_page = 0;
light_raw_payload_bytes = 0;
memset(light_raw_page_buffer, 0xFF, NAND_PAGE_SIZE_BYTES);
```

I byte della pagina precedente non vengono conteggiati nella pagina successiva.

## 8. Verifica readback

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

In caso di mismatch incrementa:

```c
light_nand_verify_failures
```

e ritorna `LOG_ERR_NAND`.

## 9. Conteggio pagine USB

Il framing USB resta invariato:

```text
LOGSTART
uint32_t total_pages
total_pages * 4096 byte
LOGEND!!
```

`NANDLogger_DownloadAll()` chiama `NANDLogger_FlushAll()` prima di leggere `logger->page_sequence`, quindi include anche l'ultima pagina `LRAW` parziale.

`page_sequence` aumenta solo se `spi_nand_page_program()` ritorna `SPI_NAND_RET_OK`. Pagine non scritte o scritte con errore NAND non vengono conteggiate come pagine valide.

## 10. Risultati dei test

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
| 144 record | 144 | 4032 | pagina non supera capacita' |
| 145 record | 145 | 4060 | pagina piena flushata |
| 146 record | 145 + 1 | 4060 + 28 | record 146 in nuova pagina |

Readback hardware: implementato nel firmware tramite `NAND_VERIFY_LRAW_AFTER_WRITE`; non verificabile fisicamente da Codex senza board collegata in esecuzione.

## 11. Risultato compilazione

Comando:

```bash
cmake --build --preset Debug
```

Risultato: compilazione e link completati con successo.

Warning rimasti:

- `Core/Src/SPI_NAND.c:488`: `-Wdiscarded-qualifiers` su `spi_write()`;
- `Core/Src/SPI_NAND.c:48`: `current_state` definita ma non usata.

Sono warning preesistenti/esterni alla fix `LRAW`.

## 12. Problemi aperti

- La conferma finale del readback richiede esecuzione su hardware reale.
- Il parser host deve comunque rispettare `payload_bytes` dell'header e non leggere sempre 4080 byte.
- La NAND viene inviata via USB a pagine complete da 4096 byte: questo e' corretto; la validita' dei dati interni e' definita solo da `payload_bytes`.

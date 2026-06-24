# AS7341 SMUX Acquisition Fix

## Problema osservato

La pipeline LRAW NAND e il download USB risultano corretti: le pagine vengono
scritte complete, il formato binario resta valido e i record non sono piu'
troncati a `0xFF`.

Il problema residuo riguarda i valori AS7341: con sensore fermo e lampada
warm-white alcuni canali risultavano molto piu' variabili o molto piu' alti di
canali adiacenti, con Clear molto piu' basso di NIR. Questo pattern e'
compatibile con dati letti da una configurazione SMUX precedente o da una
integrazione non ancora completata.

## Percorso reale dell'acquisizione

Percorso attivo della pipeline raw-count:

1. `TIM2` genera `sensor_tick_pending` a 100 Hz.
2. Il main loop in `STATE_ACQUISITION` consuma i tick in `ProcessSensorTick()`.
3. `LIGHT_SUBSAMPLE_TICKS = 8` richiede un campione luce ogni 8 tick.
4. `AcquireAndStoreLightRawSample()` chiama `AS7341_ReadFullSpectrum()`.
5. Il driver AS7341 esegue due fasi SMUX e due integrazioni.
6. `AcquireAndStoreLightRawSample()` copia i valori in `LightRawSampleRecord`.
7. `NANDLogger_AppendLightRawRecord()` serializza il record LRAW da 28 byte.

Codice attivo per LRAW:

- `Core/Src/main.c`: `ProcessSensorTick()`, `AcquireAndStoreLightRawSample()`;
- `Core/Src/as7341_driver.c`: `AS7341_ReadFullSpectrum()`;
- `Core/Src/Memory_operations.c`: `NANDLogger_AppendLightRawRecord()`.

Codice legacy/non usato dalla pipeline LRAW attiva:

- `AS7341_ReadChannels()`, helper Clear/NIR legacy;
- `light_metrics_mcu.c`, pipeline di normalizzazione/media/classificazione.

## Configurazione SMUX verificata

I byte SMUX presenti nel progetto corrispondono al reference Adafruit
`setup_F1F4_Clear_NIR()` e `setup_F5F8_Clear_NIR()`.

Fase low, F1-F4/Clear/NIR:

| SMUX addr | Valore |
| --- | --- |
| 0x00 | 0x30 |
| 0x01 | 0x01 |
| 0x02 | 0x00 |
| 0x03 | 0x00 |
| 0x04 | 0x00 |
| 0x05 | 0x42 |
| 0x06 | 0x00 |
| 0x07 | 0x00 |
| 0x08 | 0x50 |
| 0x09 | 0x00 |
| 0x0A | 0x00 |
| 0x0B | 0x00 |
| 0x0C | 0x20 |
| 0x0D | 0x04 |
| 0x0E | 0x00 |
| 0x0F | 0x30 |
| 0x10 | 0x01 |
| 0x11 | 0x50 |
| 0x12 | 0x00 |
| 0x13 | 0x06 |

Mapping effettivo desiderato:

- CH0 -> F1;
- CH1 -> F2;
- CH2 -> F3;
- CH3 -> F4;
- CH4 -> Clear;
- CH5 -> NIR.

Fase high, F5-F8/Clear/NIR:

| SMUX addr | Valore |
| --- | --- |
| 0x00 | 0x00 |
| 0x01 | 0x00 |
| 0x02 | 0x00 |
| 0x03 | 0x40 |
| 0x04 | 0x02 |
| 0x05 | 0x00 |
| 0x06 | 0x10 |
| 0x07 | 0x03 |
| 0x08 | 0x50 |
| 0x09 | 0x10 |
| 0x0A | 0x03 |
| 0x0B | 0x00 |
| 0x0C | 0x00 |
| 0x0D | 0x00 |
| 0x0E | 0x24 |
| 0x0F | 0x00 |
| 0x10 | 0x00 |
| 0x11 | 0x50 |
| 0x12 | 0x00 |
| 0x13 | 0x06 |

Mapping effettivo desiderato:

- CH0 -> F5;
- CH1 -> F6;
- CH2 -> F7;
- CH3 -> F8;
- CH4 -> Clear;
- CH5 -> NIR.

Non e' stato trovato uno scambio Clear/NIR nei byte SMUX: Clear resta su ADC4
e NIR resta su ADC5 in entrambe le fasi.

## Causa concreta trovata

Prima della correzione:

- `AS7341_Init()` lasciava `ENABLE = PON | SP_EN`;
- `AS7341_ReadFullSpectrum()` cambiava la SMUX mentre `SP_EN` poteva essere
  ancora attivo;
- `as7341_smux_apply()` impostava `SMUXEN`, ma non attendeva che il dispositivo
  lo azzerasse;
- `AS7341_ReadSixChannels()` poteva vedere `AVALID` gia' alto e leggere dati
  della fase precedente invece della nuova integrazione;
- se `AS7341_DetectMainsHz()` fosse stato usato, poteva lasciare `FDEN` attivo
  dopo una classificazione o un errore.

Il mapping SMUX era quindi corretto nel codice, ma non era garantito che i
registri CH0-CH5 letti appartenessero alla SMUX appena caricata.

## Sequenza finale SP_EN/SMUXEN/AVALID

Ogni fase SMUX ora esegue:

1. legge `ENABLE` per diagnostica;
2. disabilita `SP_EN`, `FDEN` e `SMUXEN`, lasciando `PON`;
3. attende che `SMUXEN` sia zero;
4. seleziona il register bank SMUX;
5. scrive i 20 byte SMUX;
6. torna al bank dei registri standard;
7. imposta `CFG6` con `AS7341_SMUX_CMD_WRITE`;
8. imposta `SMUXEN`;
9. attende il clear reale di `SMUXEN`;
10. abilita `SP_EN`, con `FDEN` disabilitato;
11. attende `STATUS2.AVALID`;
12. legge in burst i 12 byte `CH0_L..CH5_H` (`0x95..0xA0`);
13. decodifica little-endian;
14. disabilita `SP_EN` prima della fase successiva.

Timeout introdotti:

- `AS7341_SMUX_TIMEOUT_MS = 100`;
- `AS7341_INTEGRATION_TIMEOUT_MS = 150`.

In caso di errore o timeout il campione viene scartato e non viene scritto un
record LRAW parzialmente aggiornato.

## Register bank

La SMUX viene scritta nel bank SMUX tramite `CFG0.REGBANK`.
Prima di leggere `CH0..CH5` il driver torna al bank standard. Le letture ADC
usano i registri standard `0x95..0xA0`, quindi non mischiano dati da bank
diversi.

## ADC5 e flicker

Durante la lettura multispettrale normale:

- `FDEN` viene forzato a zero;
- `SP_EN` viene abilitato solo durante l'integrazione;
- ADC5 resta usato per NIR nella SMUX low e high.

`AS7341_DetectMainsHz()` ora disabilita `SP_EN` e `FDEN` anche prima di
ritornare dopo una misura flicker.

## File modificati

- `Core/Src/as7341_driver.c`
- `Core/Inc/as7341_driver.h`
- `AS7341_SMUX_ACQUISITION_FIX.md`

Non sono stati modificati formato LRAW, NAND, USB, parser Python, IMU,
microfono, BLE, state machine, LED o file `.ioc`.

## Diagnostica aggiunta

La diagnostica e' controllata da:

```c
#define AS7341_ENABLE_SMUX_DIAGNOSTICS 1U
```

Variabili principali da osservare:

- `raw_smux_low_ch[6]`;
- `raw_smux_high_ch[6]`;
- `final_f1` ... `final_f8`;
- `final_clear`;
- `final_nir`;
- `as7341_diag_enable_before_low_smux`;
- `as7341_diag_enable_after_low_smux`;
- `as7341_diag_enable_after_low_start`;
- `as7341_diag_enable_before_high_smux`;
- `as7341_diag_enable_after_high_smux`;
- `as7341_diag_enable_after_high_start`;
- `as7341_diag_status`;
- `as7341_diag_status2`;
- `as7341_diag_cfg0`;
- `as7341_diag_cfg1`;
- `as7341_diag_fden`;
- `as7341_diag_smux_timeout_count`;
- `as7341_diag_integration_timeout_count`;
- `as7341_diag_i2c_error_count`;
- `as7341_diag_discarded_sample_count`;
- `as7341_diag_completed_acquisition_count`.

Valori attesi dopo campioni validi:

- `as7341_diag_completed_acquisition_count` cresce;
- i timeout restano a zero;
- `as7341_diag_i2c_error_count` resta stabile;
- `as7341_diag_fden == 0`;
- gli `enable_after_*_start` hanno `SP_EN` impostato durante l'integrazione;
- `raw_smux_low_ch[4]` e `raw_smux_high_ch[4]` sono i Clear di fase;
- `raw_smux_low_ch[5]` e `raw_smux_high_ch[5]` sono i NIR di fase.

## Procedura di verifica sulla board

1. Flashare il firmware.
2. Avviare l'acquisizione con USER BUTTON.
3. Osservare in debugger:
   - `as7341_diag_completed_acquisition_count`;
   - `as7341_diag_smux_timeout_count`;
   - `as7341_diag_integration_timeout_count`;
   - `as7341_diag_i2c_error_count`;
   - `raw_smux_low_ch`;
   - `raw_smux_high_ch`;
   - `final_f1` ... `final_nir`.
4. Fermare l'acquisizione con USER BUTTON.
5. Scaricare via USB come prima.
6. Verificare che il parser Python continui a leggere lo stesso formato LRAW.
7. Controllare qualitativamente che valori consecutivi, a sensore fermo, non
   alternino dati chiaramente appartenenti a fasi SMUX diverse.

Se compaiono timeout o errori I2C, acquisire i valori diagnostici sopra elencati
prima di ulteriori modifiche.

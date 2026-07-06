# Microphone Recording Implementation Guide

This project implements digital microphone recording (typically from a PDM microphone) using the **MDF (Multi-Function Digital Filter)** peripheral on the STM32U5 series, coupled with **DMA (Direct Memory Access)**.

This guide outlines how to replicate this exact setup in another STM32 project.

---

## 1. Hardware Pin Mapping & DMA Linking (MspInit)

Before the filter can do any work, the physical pins on the microcontroller must be routed to the MDF peripheral, and the DMA channel must be linked. This happens inside `stm32u5xx_hal_msp.c` within the `HAL_MDF_MspInit()` function.

If you skip this step, your code will run, but you will only record zeroes!

```c
void HAL_MDF_MspInit(MDF_HandleTypeDef* hmdf)
{
    // 1. Enable Clocks
    __HAL_RCC_MDF1_CLK_ENABLE();
    __HAL_RCC_GPDMA1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // 2. Map the Data Line (SDI0) to PB1
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_MDF1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 3. Map the Clock Line (CCK0) to PB8
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Alternate = GPIO_AF5_MDF1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 4. Link the DMA Channel to the MDF Filter
    handle_GPDMA1_Channel0.Instance = GPDMA1_Channel0;
    handle_GPDMA1_Channel0.Init.Request = GPDMA1_REQUEST_MDF1_FLT0;
    handle_GPDMA1_Channel0.Init.Direction = DMA_PERIPH_TO_MEMORY;
    // ... (Configure DMA FIFO, Data Width to HalfWord/16-bit) ...
    HAL_DMA_Init(&handle_GPDMA1_Channel0);

    __HAL_LINKDMA(hmdf, hdma, handle_GPDMA1_Channel0);
}
```

---

## 2. Hardware Initialization (MDF & Filter)

The peripheral is initialized via STM32CubeMX generated code (`MX_MDF1_Init`), which configures both the MDF hardware interface and the digital filter used to convert the raw bitstream into usable PCM audio.

### MDF Handle Configuration
The hardware interface is configured to generate the clock for the microphone and capture the incoming bitstream:
```c
MdfHandle0.Instance = MDF1_Filter0;
MdfHandle0.Init.CommonParam.InterleavedFilters = 0;
MdfHandle0.Init.CommonParam.ProcClockDivider = 1;

// Clock Generation (Master Mode)
MdfHandle0.Init.CommonParam.OutputClock.Activation = ENABLE;
MdfHandle0.Init.CommonParam.OutputClock.Pins = MDF_OUTPUT_CLOCK_ALL;
MdfHandle0.Init.CommonParam.OutputClock.Divider = 5;

// Interface Settings
MdfHandle0.Init.SerialInterface.Activation = ENABLE;
MdfHandle0.Init.SerialInterface.Mode = MDF_SITF_NORMAL_SPI_MODE;
MdfHandle0.Init.SerialInterface.ClockSource = MDF_SITF_CCK0_SOURCE;
MdfHandle0.Init.SerialInterface.Threshold = 31;
MdfHandle0.Init.FilterBistream = MDF_BITSTREAM0_RISING;

HAL_MDF_Init(&MdfHandle0);
```

### Digital Filter Configuration
The raw PDM signal needs to be decimated and filtered to become 16-bit PCM data. This is configured in `MdfFilterConfig0`:
```c
MdfFilterConfig0.DataSource = MDF_DATA_SOURCE_BSMX;
MdfFilterConfig0.CicMode = MDF_ONE_FILTER_SINC5;           // Sinc^5 Filter
MdfFilterConfig0.DecimationRatio = 16;                     // Downsampling ratio

// Post-processing filters to improve audio quality
MdfFilterConfig0.ReshapeFilter.Activation = ENABLE;
MdfFilterConfig0.ReshapeFilter.DecimationRatio = MDF_RSF_DECIMATION_RATIO_4;
MdfFilterConfig0.HighPassFilter.Activation = ENABLE;       // Removes DC offset
MdfFilterConfig0.HighPassFilter.CutOffFrequency = MDF_HPF_CUTOFF_0_000625FPCM;

// Acquisition Settings
MdfFilterConfig0.AcquisitionMode = MDF_MODE_ASYNC_CONT;    // Continuous capture
MdfFilterConfig0.FifoThreshold = MDF_FIFO_THRESHOLD_NOT_EMPTY;
MdfFilterConfig0.DiscardSamples = AUDIO_HARDWARE_WARMUP_SAMPLES; // Discard initial noise pop
```

> [!NOTE]
> `AUDIO_HARDWARE_WARMUP_SAMPLES` is crucial. Digital microphones often have a "pop" or DC spike when first powered on. The MDF hardware automatically discards these first few samples before triggering the DMA.

---

## 3. DMA Buffer Configuration

To avoid CPU overhead, audio is streamed directly from the MDF peripheral to a circular RAM buffer using DMA.

```c
// Define the DMA configuration struct
MDF_DmaConfigTypeDef mic_dma_config;

// Define a static buffer in RAM to hold the 16-bit PCM samples
int16_t audio_dma_buffer[AUDIO_DMA_BUFFER_SAMPLES]; 

// Link the buffer to the DMA configuration
mic_dma_config.Address = (uint32_t)audio_dma_buffer;
mic_dma_config.DataLength = AUDIO_DMA_BUFFER_SAMPLES * sizeof(int16_t);
mic_dma_config.MsbOnly = ENABLE; // Important: aligns the 16-bit PCM correctly
```

---

## 4. The Audio Ring Buffer & Diagnostics Reset

Before actually turning on the microphone, the code runs through a long list of `audio_diag_` and `mic_diag.` variable resets. 
Audio is a very high-bandwidth data stream. Because the main MCU might be busy doing other things (like writing to Flash or handling Bluetooth), it cannot always process the audio immediately.

To handle this, the firmware uses an **Audio Ring Buffer** (`AudioRing_Reset()`). 
When the DMA fills the `audio_dma_buffer` (halfway or fully), it triggers an interrupt. That interrupt quickly copies the PCM data into the larger Application Ring Buffer, so the DMA can keep overwriting `audio_dma_buffer` without losing data.

Before starting a new 500ms window, the system must wipe the slate clean:
```c
// Reset the application-level ring buffer so old data doesn't corrupt the new window
AudioRing_Reset();

// Reset the acoustic calculations for the new window
audio_diag_rms_z_dbfs = 0.0;
audio_diag_rms_a_dbfs = 0.0;
audio_diag_peak_dbfs = 0.0;
audio_diag_laeq_dba = 0.0;
audio_diag_environment_class = AUDIO_ENV_UNAVAILABLE;

// Reset hardware health diagnostics (To track if DMA is failing to start/stop)
audio_diag_start_failures = 0U;
audio_diag_ring_overflows = 0U;      // Tracks if the CPU was too slow and missed audio data
audio_diag_ring_max_occupancy = 0U;  // Tracks how close the buffer came to overflowing
mic_diag.dma_start_attempt_count++;
```
If `audio_diag_ring_overflows` ever goes above `0`, it means the CPU was too slow at draining the Ring Buffer, and audio samples were permanently lost.

---

## 5. Starting the Recording Window

When the state machine determines it's time to record an audio snapshot (e.g., during the 500ms `PHASE_ENV`), it executes the following sequence:

1. **Enable the Clock**: The microphone is provided clock pulses.
2. **Start the MDF & DMA**: `HAL_MDF_AcqStart_DMA` is called, which ties the filter and the DMA buffer together and begins the hardware transfer.

```c
void StartMicRecording() 
{
    // 1. Provide clock to the physical microphone
    MicrophoneClock_Enable(); 
    
    // 2. Instruct HAL to start continuous acquisition via DMA
    HAL_StatusTypeDef start_status = HAL_MDF_AcqStart_DMA(
        &MdfHandle0,
        &MdfFilterConfig0,
        &mic_dma_config
    );
    
    if (start_status == HAL_OK) {
        microphone_active = 1U;
        mic_diag.dma_start_ok_count++;
    } else {
        // Handle initialization failure
        mic_diag.dma_start_error_count++;
    }
}
```

---

## 6. Processing and Stopping

As the DMA fills `audio_dma_buffer`, it will typically trigger Half-Transfer and Full-Transfer interrupts (configured in `stm32u5xx_it.c`). Inside these callbacks, the firmware copies the PCM data out of `audio_dma_buffer` into the application-level ring buffer.

Later in the main loop, the MCU drains the ring buffer, calculates the RMS/Peak/LAeq decibel values, and classifies the environment (e.g., "Lively" or "Quiet").

Once the 500ms window finishes, the recording is forcefully stopped to save power:

```c
void StopMicRecording() 
{
    if (microphone_active) {
        // 1. Stop the MDF acquisition and DMA transfers
        HAL_MDF_AcqStop_DMA(&MdfHandle0);
        
        // 2. Cut the clock to the physical microphone to save power
        MicrophoneClock_Disable();
        
        microphone_active = 0U;
    }
}
```

> [!TIP]
> If you are replicating this on another STM32, ensure that you correctly handle the caching architecture. If your `audio_dma_buffer` is placed in D-Cache enabled memory, you **must** invalidate the cache before reading from it in your interrupt handlers, or you will read stale data!

---

## 7. Digital Signal Processing (DSP) and Feature Extraction

Instead of saving megabytes of raw PCM audio (which fills up storage quickly and poses privacy risks), the firmware passes the raw audio through a DSP pipeline (`Audio_ComputeBasicFeatures`) and boils it down into a single summary struct: **`AudioBasicFeatureDebug`**.

This struct acts as the "acoustic report card" for the 500ms window:

```c
typedef struct
{
    uint32_t window_index;
    uint32_t window_start_ms;
    uint32_t sample_count;                 // Total samples processed

    double mean_counts;                    // The DC offset of the microphone
    double rms_zero_mean_counts;           // Standard RMS volume
    double rms_zero_mean_dbfs;             // Unweighted volume in Decibels (dBFS)

    uint32_t absolute_peak_counts;         
    double peak_dbfs;                      // The loudest single sound in the window

    uint32_t clipped_sample_count;         // Did the sound exceed the mic's physical limit?
    double clipped_sample_percentage;

    double a_weighted_rms_counts;          
    double a_weighted_rms_dbfs;            
    double estimated_laeq_dba;             // Human-perceived volume (A-weighted dB)

    uint8_t environment_class;             // Categorized noise level (e.g. "Lively", "Quiet")
    
    // Health Flags (record_valid, complete, etc.)
} AudioBasicFeatureDebug;
```

### Why this is important:
1. **Privacy & Storage**: Raw audio is immediately discarded after the features are extracted.
2. **A-Weighting**: The pipeline calculates both raw decibels and **A-weighted decibels** (`estimated_laeq_dba`). A-weighting is a specialized DSP filter that mimics how the human ear hears frequencies (e.g., highly sensitive to 2kHz tones, but less sensitive to 50Hz bass rumbles). 
3. **Downstream Logging**: The `environment_class` and `estimated_laeq_dba` are the exact values packaged into the unified `0x55` BLE Packets and NAND Flash records.

---

## 8. Advanced Low-Level Optimizations

There are three deeply technical, low-level details crucial to how the system actually operates under the hood that are worth noting for advanced integration:

### A. Direct Register Manipulation for the Clock
If you look closely at the `MicrophoneClock_Enable()` and `MicrophoneClock_Disable()` functions, you won't see STM32 HAL functions. Instead, the firmware writes directly to the silicon registers:
```c
static void MicrophoneClock_Enable(void) {
    MDF1->CKGCR |= MDF_CKGCR_CKDEN;                  // Instantly turns on the clock generator
    MdfHandle0.Instance->SITFCR |= MDF_SITFCR_SITFEN; // Instantly turns on the Serial Interface
}
```
**Why?** Using HAL functions to start and stop the MDF takes too many CPU cycles. By directly flipping the bits in the `CKGCR` and `SITFCR` registers, the MCU can turn the microphone on and off instantly at the start and end of the 500ms windows, saving precious microseconds of battery life.

### B. The DMA Interrupt Callbacks (ISR)
The DMA triggers interrupts located in `stm32u5xx_it.c`. 
When the `audio_dma_buffer` is halfway full, the hardware fires `HAL_MDF_RxHalfCpltCallback`. When it's completely full, it fires `HAL_MDF_RxCpltCallback`. 
Inside those callbacks, the CPU wakes up just long enough to copy the new PCM data into the `AudioRing_` buffer, and then immediately goes back to sleep while the DMA starts overwriting the other half of the buffer.

### C. The A-Weighting DSP Math (Biquad Filters)
There is a constant array called `audio_a_weighting_biquads`. This contains pre-calculated coefficients for a **Direct-Form II Transposed Biquad Filter**. 
To calculate `estimated_laeq_dba` (A-weighted decibels), the firmware mathematically filters the raw PCM audio using these coefficients. It artificially reduces the volume of low frequencies (bass) and boosts the volume of frequencies around 2kHz to 4kHz. This exactly mimics the physical shape and frequency response of the human ear, which is strictly required for industrial or medical noise-exposure logging.

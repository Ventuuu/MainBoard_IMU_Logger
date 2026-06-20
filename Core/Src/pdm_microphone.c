#include "pdm_microphone.h"
#include "main.h"

// Interne Variablen des Treibers (statisch)
//static MDF_HandleTypeDef MdfHandle0;
//static MDF_FilterConfigTypeDef MdfFilterConfig0;
const static int16_t AudioRecBuffer[AUDIO_REC_BUFF_SIZE];
static int16_t UartTxBuffer[AUDIO_REC_BUFF_SIZE / 2];
static UART_HandleTypeDef *p_huart_instance = NULL; // Speichert das UART-Handle für die Callbacks

// Lokale Hilfsfunktion zur Datenkonvertierung und Übertragung
static void PDM_Microphone_ProcessAndSend(uint8_t half)
{
    uint32_t startIndex = (half == 0) ? 0 : (AUDIO_REC_BUFF_SIZE / 2);
    
    // Konvertierung von 32-Bit auf 16-Bit (Bitshift für korrekte Ausrichtung)
    for (int i = 0; i < (AUDIO_REC_BUFF_SIZE / 2); i++)
    {
        UartTxBuffer[i] = (int16_t)(AudioRecBuffer[startIndex + i] >> 8);
    }
    
    // Übertragung via UART-DMA, sofern ein gültiges Handle registriert ist
    if (p_huart_instance != NULL)
    {
        HAL_UART_Transmit_DMA(p_huart_instance, (uint8_t*)UartTxBuffer, (AUDIO_REC_BUFF_SIZE / 2) * 2);
    }
}

// Initialisierung des Mikrofon-Moduls
HAL_StatusTypeDef PDM_Microphone_Init(MDF_HandleTypeDef *hmdf, MDF_FilterConfigTypeDef *MdfFilterConfig0)
{

    // Die grundlegende Konfiguration von MdfHandle0 (SITF, Filter, Takt) 
    // wurde bereits erfolgreich in deiner main.c durch MX_MDF1_Init() erledigt.
    // Wir prüfen hier nur, ob das übergebene Handle gültig ist.
    if (hmdf == NULL || MdfFilterConfig0 ==NULL) 
    {
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

// Startet die DMA-Aufnahme und registriert das UART-Handle
HAL_StatusTypeDef PDM_Microphone_Start(MDF_HandleTypeDef *hmdf, UART_HandleTypeDef *huart,MDF_DmaConfigTypeDef *Dma_config)
{
    // UART-Handle für die spätere Übertragung in den Callbacks sichern
    p_huart_instance = huart;

    // Aktiviert den Filter und startet die DMA-Akquisition im kontinuierlichen Modus.
    // Dies ist die offizielle STM32U5-HAL-Funktion für den MDF-Filter.
    return HAL_MDF_AcqStart_DMA (hmdf, 0, Dma_config);
}

// --- VERARBEITUNG DER DATEN PER INTERRUPT-CALLBACKS ---

// Half Transfer Callback: Erste Hälfte des DMA-Puffers ist voll
void HAL_MDF_AcqHalfCpltCallback(MDF_HandleTypeDef *hmdf)
{
    PDM_Microphone_ProcessAndSend(0);
}

// Complete Transfer Callback: Gesamter DMA-Puffer ist voll
void HAL_MDF_AcqCpltCallback(MDF_HandleTypeDef *hmdf)
{
    PDM_Microphone_ProcessAndSend(1);
}
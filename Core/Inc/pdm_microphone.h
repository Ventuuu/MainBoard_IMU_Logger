#ifndef PDM_MICROPHONE_H
#define PDM_MICROPHONE_H

#include "main.h" // Ermöglicht den Zugriff auf alle HAL-Typen (inkl. MDF und UART)

#define HALF_BUFFER_SIZE 512
#define FULL_BUFFER_SIZE (HALF_BUFFER_SIZE*2)
// --- Funktionen für die main.c ---

/**
 * @brief Initialisiert den Multi-Function Digital Filter (MDF) für das PDM-Mikrofon.
 * @param hmdf Zeiger auf das MDF-Handle
 * @retval HAL-Status (HAL_OK bei Erfolg)
 */
HAL_StatusTypeDef PDM_Microphone_Init(MDF_HandleTypeDef *hmdf, MDF_FilterConfigTypeDef *MdfFilterConfig0);

/**
 * @brief Startet die Audio-Aufnahme über DMA und registriert das UART-Handle für das Senden.
 * @param hmdf Zeiger auf das MDF-Handle
 * @param huart Zeiger auf das UART-Handle (für die automatische DMA-Übertragung im Hintergrund)
 * @retval HAL-Status (HAL_OK bei Erfolg)
 */
HAL_StatusTypeDef PDM_Microphone_Start(MDF_HandleTypeDef *hmdf, UART_HandleTypeDef *huart,MDF_DmaConfigTypeDef *Dma_config);

#endif // PDM_MICROPHONE_H
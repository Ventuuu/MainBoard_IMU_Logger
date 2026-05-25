
/*
--------- WIP | Still learning how to configure digital filters and communication -----------
*/


#include "imp34dt05_driver.h"

hal_mdf_serial_interface_config_t sitf_config;

sitf_config.mode = HAL_MDF_SITF_MODE_SPI_FALLING;
sitf_config.clock_source = HAL_MDF_SITF_CK_SRC_EXTERNAL;
sitf_config.threshold = 16;



CLOCK_DIVIDER = 64 //move in header
imp34dt05_clock_div = CLOCK_DIVIDER

void imp34dt05_init()
{
    if(HAL_MDF_Init(MdfHandle0,MdfFilterConfig0)  != HAL_OK)
    {
        Error_Handler();
    }
    if(HAL_MDF_SetConfig (MdfHandle0, imp34dt05_clock_div) != HAL_OK)
    {
        Error_Handler();
    }
    if(HAL_MDF_Start (MdfHandle0) != HAL_OK)
    {
        Error_Handler();
    }
    // Serial interface
    if(HAL_MDF_SITF_SetConfig(MdfHandle0,HAL_MDF_BLOCK0,&sitf_config) != HAL_OK)
    {
        Error_Handler();
    }
    if(HAL_MDF_SITF_Start (MdfHandle0, &sitf_config) != HAL_OK)
    {
        Error_Handler();
    }
    // Bitstream matrix
    if(HAL_MDF_BSMX_SetConfig(MdfHandle0,HAL_MDF_BLOCK0,HAL_MDF_BSMX_INPUT0_RISING) () != HAL_OK)
    {
        Error_Handler();
    }
    //short-circuit detector
    /*
    if(HAL_MDF_SCD_SetConfig(MdfHandle0, ) != HAL_OK)
    {
        Error_Handler();
    }
        */

    // Digital filter
    if(HAL_MDF_DFLT_SetConfig(MdfHandle0) != HAL_OK)
    {
        Error_Handler;
    }
    
}

void imp34dt05_readHalfBuffer()
{
    HAL_MDF_DFLT_StartAcq_IT();
}
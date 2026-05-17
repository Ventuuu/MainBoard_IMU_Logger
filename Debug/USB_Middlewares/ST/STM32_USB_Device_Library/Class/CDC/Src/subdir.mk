################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../USB_Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c 

OBJS += \
./USB_Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.o 

C_DEPS += \
./USB_Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.d 


# Each subdirectory must supply rules for building sources it contributes
USB_Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/%.o USB_Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/%.su USB_Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/%.cyclo: ../USB_Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/%.c USB_Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U575xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/ila10/OneDrive - Politecnico di Milano/Documenti/PoliMi/SEL/Smart Wearables/MainBoard_IMU_Logger/USB_Device" -I"C:/Users/ila10/OneDrive - Politecnico di Milano/Documenti/PoliMi/SEL/Smart Wearables/MainBoard_IMU_Logger/USB_Middlewares" -I"C:/Users/ila10/OneDrive - Politecnico di Milano/Documenti/PoliMi/SEL/Smart Wearables/MainBoard_IMU_Logger/USB_Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Users/ila10/OneDrive - Politecnico di Milano/Documenti/PoliMi/SEL/Smart Wearables/MainBoard_IMU_Logger/USB_Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/ila10/OneDrive - Politecnico di Milano/Documenti/PoliMi/SEL/Smart Wearables/MainBoard_IMU_Logger/USB_Device/Target" -I"C:/Users/ila10/OneDrive - Politecnico di Milano/Documenti/PoliMi/SEL/Smart Wearables/MainBoard_IMU_Logger/USB_Device/App" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-USB_Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Class-2f-CDC-2f-Src

clean-USB_Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Class-2f-CDC-2f-Src:
	-$(RM) ./USB_Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.cyclo ./USB_Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.d ./USB_Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.o ./USB_Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.su

.PHONY: clean-USB_Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Class-2f-CDC-2f-Src


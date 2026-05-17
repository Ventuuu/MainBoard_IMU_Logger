################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../USB_Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
../USB_Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
../USB_Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c 

OBJS += \
./USB_Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.o \
./USB_Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.o \
./USB_Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.o 

C_DEPS += \
./USB_Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.d \
./USB_Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.d \
./USB_Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.d 


# Each subdirectory must supply rules for building sources it contributes
USB_Middlewares/ST/STM32_USB_Device_Library/Core/Src/%.o USB_Middlewares/ST/STM32_USB_Device_Library/Core/Src/%.su USB_Middlewares/ST/STM32_USB_Device_Library/Core/Src/%.cyclo: ../USB_Middlewares/ST/STM32_USB_Device_Library/Core/Src/%.c USB_Middlewares/ST/STM32_USB_Device_Library/Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U575xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/ila10/OneDrive - Politecnico di Milano/Documenti/PoliMi/SEL/Smart Wearables/MainBoard_IMU_Logger/USB_Device" -I"C:/Users/ila10/OneDrive - Politecnico di Milano/Documenti/PoliMi/SEL/Smart Wearables/MainBoard_IMU_Logger/USB_Middlewares" -I"C:/Users/ila10/OneDrive - Politecnico di Milano/Documenti/PoliMi/SEL/Smart Wearables/MainBoard_IMU_Logger/USB_Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Users/ila10/OneDrive - Politecnico di Milano/Documenti/PoliMi/SEL/Smart Wearables/MainBoard_IMU_Logger/USB_Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/ila10/OneDrive - Politecnico di Milano/Documenti/PoliMi/SEL/Smart Wearables/MainBoard_IMU_Logger/USB_Device/Target" -I"C:/Users/ila10/OneDrive - Politecnico di Milano/Documenti/PoliMi/SEL/Smart Wearables/MainBoard_IMU_Logger/USB_Device/App" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-USB_Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Core-2f-Src

clean-USB_Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Core-2f-Src:
	-$(RM) ./USB_Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.cyclo ./USB_Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.d ./USB_Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.o ./USB_Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.su ./USB_Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.cyclo ./USB_Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.d ./USB_Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.o ./USB_Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.su ./USB_Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.cyclo ./USB_Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.d ./USB_Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.o ./USB_Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.su

.PHONY: clean-USB_Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Core-2f-Src


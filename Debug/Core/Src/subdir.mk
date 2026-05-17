################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Memory_operations.c \
../Core/Src/SPI_NAND.c \
../Core/Src/bluetooth.c \
../Core/Src/imu_driver.c \
../Core/Src/led_driver.c \
../Core/Src/main.c \
../Core/Src/stm32u5xx_hal_msp.c \
../Core/Src/stm32u5xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32u5xx.c 

OBJS += \
./Core/Src/Memory_operations.o \
./Core/Src/SPI_NAND.o \
./Core/Src/bluetooth.o \
./Core/Src/imu_driver.o \
./Core/Src/led_driver.o \
./Core/Src/main.o \
./Core/Src/stm32u5xx_hal_msp.o \
./Core/Src/stm32u5xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32u5xx.o 

C_DEPS += \
./Core/Src/Memory_operations.d \
./Core/Src/SPI_NAND.d \
./Core/Src/bluetooth.d \
./Core/Src/imu_driver.d \
./Core/Src/led_driver.d \
./Core/Src/main.d \
./Core/Src/stm32u5xx_hal_msp.d \
./Core/Src/stm32u5xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32u5xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U575xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/ila10/OneDrive - Politecnico di Milano/Documenti/PoliMi/SEL/Smart Wearables/MainBoard_IMU_Logger/USB_Device" -I"C:/Users/ila10/OneDrive - Politecnico di Milano/Documenti/PoliMi/SEL/Smart Wearables/MainBoard_IMU_Logger/USB_Middlewares" -I"C:/Users/ila10/OneDrive - Politecnico di Milano/Documenti/PoliMi/SEL/Smart Wearables/MainBoard_IMU_Logger/USB_Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Users/ila10/OneDrive - Politecnico di Milano/Documenti/PoliMi/SEL/Smart Wearables/MainBoard_IMU_Logger/USB_Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/ila10/OneDrive - Politecnico di Milano/Documenti/PoliMi/SEL/Smart Wearables/MainBoard_IMU_Logger/USB_Device/Target" -I"C:/Users/ila10/OneDrive - Politecnico di Milano/Documenti/PoliMi/SEL/Smart Wearables/MainBoard_IMU_Logger/USB_Device/App" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Memory_operations.cyclo ./Core/Src/Memory_operations.d ./Core/Src/Memory_operations.o ./Core/Src/Memory_operations.su ./Core/Src/SPI_NAND.cyclo ./Core/Src/SPI_NAND.d ./Core/Src/SPI_NAND.o ./Core/Src/SPI_NAND.su ./Core/Src/bluetooth.cyclo ./Core/Src/bluetooth.d ./Core/Src/bluetooth.o ./Core/Src/bluetooth.su ./Core/Src/imu_driver.cyclo ./Core/Src/imu_driver.d ./Core/Src/imu_driver.o ./Core/Src/imu_driver.su ./Core/Src/led_driver.cyclo ./Core/Src/led_driver.d ./Core/Src/led_driver.o ./Core/Src/led_driver.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32u5xx_hal_msp.cyclo ./Core/Src/stm32u5xx_hal_msp.d ./Core/Src/stm32u5xx_hal_msp.o ./Core/Src/stm32u5xx_hal_msp.su ./Core/Src/stm32u5xx_it.cyclo ./Core/Src/stm32u5xx_it.d ./Core/Src/stm32u5xx_it.o ./Core/Src/stm32u5xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32u5xx.cyclo ./Core/Src/system_stm32u5xx.d ./Core/Src/system_stm32u5xx.o ./Core/Src/system_stm32u5xx.su

.PHONY: clean-Core-2f-Src


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32u575ritxq.s 

OBJS += \
./Core/Startup/startup_stm32u575ritxq.o 

S_DEPS += \
./Core/Startup/startup_stm32u575ritxq.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m33 -g3 -DDEBUG -c -I"C:/Users/ila10/OneDrive - Politecnico di Milano/Documenti/PoliMi/SEL/Smart Wearables/MainBoard_IMU_Logger/USB_Device" -I"C:/Users/ila10/OneDrive - Politecnico di Milano/Documenti/PoliMi/SEL/Smart Wearables/MainBoard_IMU_Logger/USB_Middlewares" -I"C:/Users/ila10/OneDrive - Politecnico di Milano/Documenti/PoliMi/SEL/Smart Wearables/MainBoard_IMU_Logger/USB_Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Users/ila10/OneDrive - Politecnico di Milano/Documenti/PoliMi/SEL/Smart Wearables/MainBoard_IMU_Logger/USB_Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/ila10/OneDrive - Politecnico di Milano/Documenti/PoliMi/SEL/Smart Wearables/MainBoard_IMU_Logger/USB_Device/Target" -I"C:/Users/ila10/OneDrive - Politecnico di Milano/Documenti/PoliMi/SEL/Smart Wearables/MainBoard_IMU_Logger/USB_Device/App" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32u575ritxq.d ./Core/Startup/startup_stm32u575ritxq.o

.PHONY: clean-Core-2f-Startup


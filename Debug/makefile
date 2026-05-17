################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include USB_Middlewares/ST/STM32_USB_Device_Library/Core/Src/subdir.mk
-include USB_Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/subdir.mk
-include USB_Device/Target/subdir.mk
-include USB_Device/App/subdir.mk
-include Drivers/STM32U5xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := MainBoard_IMU_Logger
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
MainBoard_IMU_Logger.elf \

MAP_FILES += \
MainBoard_IMU_Logger.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
MainBoard_IMU_Logger.list \


# All Target
all: main-build

# Main-build Target
main-build: MainBoard_IMU_Logger.elf secondary-outputs

# Tool invocations
MainBoard_IMU_Logger.elf MainBoard_IMU_Logger.map: $(OBJS) $(USER_OBJS) C:\Users\ila10\OneDrive\ -\ Politecnico\ di\ Milano\Documenti\PoliMi\SEL\Smart\ Wearables\MainBoard_IMU_Logger\STM32U575RITXQ_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "MainBoard_IMU_Logger.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m33 -T"C:\Users\ila10\OneDrive - Politecnico di Milano\Documenti\PoliMi\SEL\Smart Wearables\MainBoard_IMU_Logger\STM32U575RITXQ_FLASH.ld" --specs=nosys.specs -Wl,-Map="MainBoard_IMU_Logger.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

MainBoard_IMU_Logger.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "MainBoard_IMU_Logger.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) MainBoard_IMU_Logger.elf MainBoard_IMU_Logger.list MainBoard_IMU_Logger.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets

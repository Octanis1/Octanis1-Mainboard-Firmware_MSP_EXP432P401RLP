################################################################################
# Automatically-generated file. Do not edit!
################################################################################

CG_TOOL_ROOT := /home/vagrant/ti/ccsv6/tools/compiler/gcc-arm-none-eabi-4_8-2014q3

ORDERED_OBJS += \
"./MSP_EXP432P401RLP.o" \
"./fw/tests/minunit.o" \
"./fw/peripherals/compass.o" \
"./fw/peripherals/flash.o" \
"./fw/peripherals/gps.o" \
"./fw/peripherals/imu.o" \
"./fw/peripherals/motor.o" \
"./fw/peripherals/radio.o" \
"./fw/peripherals/rockblock.o" \
"./fw/peripherals/sonar.o" \
"./fw/peripherals/strut.o" \
"./fw/peripherals/weather.o" \
"./fw/peripherals/wheel.o" \
"./fw/peripherals/hal/mpu6050.o" \
"./fw/nav/drive.o" \
"./fw/nav/map.o" \
"./fw/nav/path.o" \
"./fw/nav/pose.o" \
"./fw/main.o" \
"./fw/lib/minmea/minmea.o" \
"./fw/core/cli.o" \
"./fw/core/log.o" \
-Wl,-T"../MSP_EXP432P401RLP.lds" \
$(GEN_CMDS__FLAG) \
-Wl,--start-group \
-l":msp432p4xx_driverlib.a" \
-l"gcc" \
-l"c" \
-l"m" \
-l"rdimon" \
-Wl,--end-group \

-include ../makefile.init

RM := rm -rf
RMDIR := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include fw/tests/subdir_vars.mk
-include fw/peripherals/subdir_vars.mk
-include fw/peripherals/hal/subdir_vars.mk
-include fw/nav/subdir_vars.mk
-include fw/subdir_vars.mk
-include fw/lib/minmea/subdir_vars.mk
-include fw/core/subdir_vars.mk
-include subdir_rules.mk
-include fw/tests/subdir_rules.mk
-include fw/peripherals/subdir_rules.mk
-include fw/peripherals/hal/subdir_rules.mk
-include fw/nav/subdir_rules.mk
-include fw/subdir_rules.mk
-include fw/lib/minmea/subdir_rules.mk
-include fw/core/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
Octanis1-Mainboard-Firmware_MSP_EXP432P401RLP.out \

EXE_OUTPUTS__QUOTED += \
"Octanis1-Mainboard-Firmware_MSP_EXP432P401RLP.out" \

BIN_OUTPUTS += \
Octanis1-Mainboard-Firmware_MSP_EXP432P401RLP.hex \

BIN_OUTPUTS__QUOTED += \
"Octanis1-Mainboard-Firmware_MSP_EXP432P401RLP.hex" \


# All Target
all: Octanis1-Mainboard-Firmware_MSP_EXP432P401RLP.out

# Tool invocations
Octanis1-Mainboard-Firmware_MSP_EXP432P401RLP.out: $(OBJS) $(LDS_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU Linker'
	"/home/vagrant/ti/ccsv6/tools/compiler/gcc-arm-none-eabi-4_8-2014q3/bin/arm-none-eabi-gcc" -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__MSP432P401R__ -DTARGET_IS_FALCON -Dgcc -DMSP432WARE -Dtimegm=mktime -O0 -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -Wall -Wl,-Map,"Octanis1-Mainboard-Firmware_MSP_EXP432P401RLP.map" -nostartfiles -static -Wl,--gc-sections -L"/home/vagrant/ti/tirtos_msp43x_2_12_00_24/products/MSPWare_2_00_00_40a/driverlib/MSP432P4xx/gcc" -L"/home/vagrant/ti/tirtos_msp43x_2_12_00_24/products/bios_6_41_03_51/packages/gnu/targets/arm/libs/install-native/arm-none-eabi/lib/armv7e-m/fpu" -mthumb -g -gstrict-dwarf -Wall -o"Octanis1-Mainboard-Firmware_MSP_EXP432P401RLP.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

Octanis1-Mainboard-Firmware_MSP_EXP432P401RLP.hex: $(EXE_OUTPUTS)
	@echo 'Invoking: GNU Objcopy Utility'
	"/home/vagrant/ti/ccsv6/tools/compiler/gcc-arm-none-eabi-4_8-2014q3/bin/arm-none-eabi-objcopy"  $(EXE_OUTPUTS__QUOTED) "Octanis1-Mainboard-Firmware_MSP_EXP432P401RLP.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(GEN_MISC_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)$(GEN_FILES__QUOTED)$(BIN_OUTPUTS__QUOTED)$(GEN_OPTS__QUOTED)$(GEN_CMDS__QUOTED)
	-$(RMDIR) $(GEN_MISC_DIRS__QUOTED)
	-$(RM) "MSP_EXP432P401RLP.d" "fw/tests/minunit.d" "fw/peripherals/compass.d" "fw/peripherals/flash.d" "fw/peripherals/gps.d" "fw/peripherals/imu.d" "fw/peripherals/motor.d" "fw/peripherals/radio.d" "fw/peripherals/rockblock.d" "fw/peripherals/sonar.d" "fw/peripherals/strut.d" "fw/peripherals/weather.d" "fw/peripherals/wheel.d" "fw/peripherals/hal/mpu6050.d" "fw/nav/drive.d" "fw/nav/map.d" "fw/nav/path.d" "fw/nav/pose.d" "fw/main.d" "fw/lib/minmea/minmea.d" "fw/core/cli.d" "fw/core/log.d" 
	-$(RM) "MSP_EXP432P401RLP.o" "fw/tests/minunit.o" "fw/peripherals/compass.o" "fw/peripherals/flash.o" "fw/peripherals/gps.o" "fw/peripherals/imu.o" "fw/peripherals/motor.o" "fw/peripherals/radio.o" "fw/peripherals/rockblock.o" "fw/peripherals/sonar.o" "fw/peripherals/strut.o" "fw/peripherals/weather.o" "fw/peripherals/wheel.o" "fw/peripherals/hal/mpu6050.o" "fw/nav/drive.o" "fw/nav/map.o" "fw/nav/path.o" "fw/nav/pose.o" "fw/main.o" "fw/lib/minmea/minmea.o" "fw/core/cli.o" "fw/core/log.o" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets


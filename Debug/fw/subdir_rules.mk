################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
fw/%.o: ../fw/%.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Compiler'
	"C:/ti/ccsv6/tools/compiler/gcc-arm-none-eabi-4_8-2014q3/bin/arm-none-eabi-gcc.exe" -c -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__MSP432P401R__ -DTARGET_IS_FALCON -Dgcc -DMSP432WARE -Dtimegm=mktime -I"C:/ti/ccsv6/tools/compiler/gcc-arm-none-eabi-4_8-2014q3/arm-none-eabi/include" -I"C:/ti/ccsv6/ccs_base/arm/include" -I"C:/ti/ccsv6/ccs_base/arm/include/CMSIS" -I"C:/ti/tirtos_msp43x_2_12_01_33/products/MSPWare_2_00_00_40a/driverlib/MSP432P4xx" -O0 -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -Wall -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -std=c99 -o"$@" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '



################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Compiler'
	"/home/vagrant/ti/ccsv6/tools/compiler/gcc-arm-none-eabi-4_8-2014q3/bin/arm-none-eabi-gcc" -c -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__MSP432P401R__ -DTARGET_IS_FALCON -Dgcc -DMSP432WARE -Dtimegm=mktime -I"/home/vagrant/ti/ccsv6/tools/compiler/gcc-arm-none-eabi-4_8-2014q3/arm-none-eabi/include" -I"/home/vagrant/ti/ccsv6/ccs_base/arm/include" -I"/home/vagrant/ti/ccsv6/ccs_base/arm/include/CMSIS" -I"/home/vagrant/ti/tirtos_msp43x_2_12_01_33/products/MSPWare_2_00_00_40a/driverlib/MSP432P4xx" -O0 -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -Wall -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -std=c99 -o"$@" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

configPkg/linker.cmd: ../app.cfg
	@echo 'Building file: $<'
	@echo 'Invoking: XDCtools'
	"/home/vagrant/ti/xdctools_3_31_00_24_core/xs" --xdcpath="/home/vagrant/ti/tirtos_msp43x_2_12_01_33/packages;/home/vagrant/ti/tirtos_msp43x_2_12_01_33/products/bios_6_41_04_54/packages;/home/vagrant/ti/tirtos_msp43x_2_12_01_33/products/uia_2_00_02_39/packages;/home/vagrant/ti/ccsv6/ccs_base;" xdc.tools.configuro -o configPkg -t gnu.targets.arm.M4F -p ti.platforms.msp432:MSP432P401R -r release -c "/home/vagrant/ti/ccsv6/tools/compiler/gcc-arm-none-eabi-4_8-2014q3" --compileOptions "-mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__MSP432P401R__ -DTARGET_IS_FALCON -Dgcc -DMSP432WARE -Dtimegm=mktime -I\"/home/vagrant/ti/ccsv6/tools/compiler/gcc-arm-none-eabi-4_8-2014q3/arm-none-eabi/include\" -I\"/home/vagrant/ti/ccsv6/ccs_base/arm/include\" -I\"/home/vagrant/ti/ccsv6/ccs_base/arm/include/CMSIS\" -I\"/home/vagrant/ti/tirtos_msp43x_2_12_01_33/products/MSPWare_2_00_00_40a/driverlib/MSP432P4xx\" -O0 -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -Wall  " "$<"
	@echo 'Finished building: $<'
	@echo ' '

configPkg/compiler.opt: | configPkg/linker.cmd
configPkg/: | configPkg/linker.cmd



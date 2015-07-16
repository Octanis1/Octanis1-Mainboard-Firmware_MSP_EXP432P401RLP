#
#  Do not edit this file.  This file is generated from 
#  package.bld.  Any modifications to this file will be 
#  overwritten whenever makefiles are re-generated.
#
#  target compatibility key = gnu.targets.arm.M4F{1,0,4.8,4
#
ifeq (,$(MK_NOGENDEPS))
-include package/cfg/app_pm4fg.om4fg.dep
package/cfg/app_pm4fg.om4fg.dep: ;
endif

package/cfg/app_pm4fg.om4fg: | .interfaces
package/cfg/app_pm4fg.om4fg: package/cfg/app_pm4fg.c package/cfg/app_pm4fg.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clm4fg $< ...
	$(gnu.targets.arm.M4F.rootDir)/bin/arm-none-eabi-gcc -c -MD -MF $@.dep -x c  -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__MSP432P401R__ -DTARGET_IS_FALCON -Dgcc -DMSP432WARE -I\"/home/vagrant/ti/ccsv6/tools/compiler/gcc-arm-none-eabi-4_8-2014q3/arm-none-eabi/include\" -I\"/home/vagrant/ti/ccsv6/ccs_base/arm/include\" -I\"/home/vagrant/ti/ccsv6/ccs_base/arm/include/CMSIS\" -I\"/home/vagrant/ti/tirtos_msp43x_2_12_00_24/products/MSPWare_2_00_00_40a/driverlib/MSP432P4xx\" -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -Wall   -Wunused -Wunknown-pragmas -ffunction-sections -fdata-sections  -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mabi=aapcs -g -Dfar= -D__DYNAMIC_REENT__   -Dxdc_cfg__xheader__='"xconfig_app/package/cfg/app_pm4fg.h"'  -Dxdc_target_name__=M4F -Dxdc_target_types__=gnu/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_4_8_4  -O2  -I/home/vagrant/ti/tirtos_msp43x_2_12_00_24/products/bios_6_41_03_51/packages/gnu/targets/arm//libs/install-native/arm-none-eabi/include   $(XDCINCS)  -o $@ $<
	-@$(FIXDEP) $@.dep $@.dep
	
package/cfg/app_pm4fg.om4fg: export LD_LIBRARY_PATH=

package/cfg/app_pm4fg.sm4fg: | .interfaces
package/cfg/app_pm4fg.sm4fg: package/cfg/app_pm4fg.c package/cfg/app_pm4fg.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clm4fg -S $< ...
	$(gnu.targets.arm.M4F.rootDir)/bin/arm-none-eabi-gcc -c -MD -MF $@.dep -x c -S -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__MSP432P401R__ -DTARGET_IS_FALCON -Dgcc -DMSP432WARE -I\"/home/vagrant/ti/ccsv6/tools/compiler/gcc-arm-none-eabi-4_8-2014q3/arm-none-eabi/include\" -I\"/home/vagrant/ti/ccsv6/ccs_base/arm/include\" -I\"/home/vagrant/ti/ccsv6/ccs_base/arm/include/CMSIS\" -I\"/home/vagrant/ti/tirtos_msp43x_2_12_00_24/products/MSPWare_2_00_00_40a/driverlib/MSP432P4xx\" -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -Wall   -Wunused -Wunknown-pragmas -ffunction-sections -fdata-sections  -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mabi=aapcs -g -Dfar= -D__DYNAMIC_REENT__   -Dxdc_cfg__xheader__='"xconfig_app/package/cfg/app_pm4fg.h"'  -Dxdc_target_name__=M4F -Dxdc_target_types__=gnu/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_4_8_4  -O2   $(XDCINCS)  -o $@ $<
	-@$(FIXDEP) $@.dep $@.dep
	
package/cfg/app_pm4fg.sm4fg: export LD_LIBRARY_PATH=

clean,m4fg ::
	-$(RM) package/cfg/app_pm4fg.om4fg
	-$(RM) package/cfg/app_pm4fg.sm4fg

app.pm4fg: package/cfg/app_pm4fg.om4fg package/cfg/app_pm4fg.mak

clean::
	-$(RM) package/cfg/app_pm4fg.mak

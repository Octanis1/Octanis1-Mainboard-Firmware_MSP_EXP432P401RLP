################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
LDS_SRCS += \
../MSP_EXP432P401RLP.lds 

CFG_SRCS += \
../app.cfg 

C_SRCS += \
../MSP_EXP432P401RLP.c 

OBJS += \
./MSP_EXP432P401RLP.o 

C_DEPS += \
./MSP_EXP432P401RLP.d 

GEN_MISC_DIRS += \
./configPkg/ 

GEN_CMDS += \
./configPkg/linker.cmd 

GEN_OPTS += \
./configPkg/compiler.opt 

GEN_FILES += \
./configPkg/linker.cmd \
./configPkg/compiler.opt 

GEN_FILES__QUOTED += \
"configPkg/linker.cmd" \
"configPkg/compiler.opt" 

GEN_MISC_DIRS__QUOTED += \
"configPkg/" 

C_DEPS__QUOTED += \
"MSP_EXP432P401RLP.d" 

OBJS__QUOTED += \
"MSP_EXP432P401RLP.o" 

C_SRCS__QUOTED += \
"../MSP_EXP432P401RLP.c" 

GEN_CMDS__FLAG += \
-Wl,-T"./configPkg/linker.cmd" 

GEN_OPTS__FLAG += \
@"./configPkg/compiler.opt" 



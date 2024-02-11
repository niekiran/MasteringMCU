################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bsp/ds1307.c \
../bsp/lcd.c 

OBJS += \
./bsp/ds1307.o \
./bsp/lcd.o 

C_DEPS += \
./bsp/ds1307.d \
./bsp/lcd.d 


# Each subdirectory must supply rules for building sources it contributes
bsp/%.o bsp/%.su bsp/%.cyclo: ../bsp/%.c bsp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I"C:/Users/Shishir Dey/Documents/Bharati Software/MasteringMCU/Resources/Source_code/Workspace/stm32f4xx_drivers/bsp" -I"C:/Users/Shishir Dey/Documents/Bharati Software/MasteringMCU/Resources/Source_code/Workspace/stm32f4xx_drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-bsp

clean-bsp:
	-$(RM) ./bsp/ds1307.cyclo ./bsp/ds1307.d ./bsp/ds1307.o ./bsp/ds1307.su ./bsp/lcd.cyclo ./bsp/lcd.d ./bsp/lcd.o ./bsp/lcd.su

.PHONY: clean-bsp


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/001led_toggle.c 

OBJS += \
./src/001led_toggle.o 

C_DEPS += \
./src/001led_toggle.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o src/%.su src/%.cyclo: ../src/%.c src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I"C:/Users/Shishir Dey/Documents/Bharati Software/MasteringMCU/Resources/Source_code/Workspace/stm32f4xx_drivers/bsp" -I"C:/Users/Shishir Dey/Documents/Bharati Software/MasteringMCU/Resources/Source_code/Workspace/stm32f4xx_drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-src

clean-src:
	-$(RM) ./src/001led_toggle.cyclo ./src/001led_toggle.d ./src/001led_toggle.o ./src/001led_toggle.su

.PHONY: clean-src


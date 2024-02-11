################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../autogen/startup_stm32f407vgtx.s 

C_SRCS += \
../autogen/syscalls.c \
../autogen/sysmem.c 

OBJS += \
./autogen/startup_stm32f407vgtx.o \
./autogen/syscalls.o \
./autogen/sysmem.o 

S_DEPS += \
./autogen/startup_stm32f407vgtx.d 

C_DEPS += \
./autogen/syscalls.d \
./autogen/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
autogen/%.o: ../autogen/%.s autogen/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"
autogen/%.o autogen/%.su autogen/%.cyclo: ../autogen/%.c autogen/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I"C:/Users/Shishir Dey/Documents/Bharati Software/MasteringMCU/Resources/Source_code/Workspace/stm32f4xx_drivers/bsp" -I"C:/Users/Shishir Dey/Documents/Bharati Software/MasteringMCU/Resources/Source_code/Workspace/stm32f4xx_drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-autogen

clean-autogen:
	-$(RM) ./autogen/startup_stm32f407vgtx.d ./autogen/startup_stm32f407vgtx.o ./autogen/syscalls.cyclo ./autogen/syscalls.d ./autogen/syscalls.o ./autogen/syscalls.su ./autogen/sysmem.cyclo ./autogen/sysmem.d ./autogen/sysmem.o ./autogen/sysmem.su

.PHONY: clean-autogen


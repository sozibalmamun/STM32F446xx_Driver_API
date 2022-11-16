################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/gpio_toggoling.c \
../Src/toggoling_with_sw.c 

OBJS += \
./Src/gpio_toggoling.o \
./Src/toggoling_with_sw.o 

C_DEPS += \
./Src/gpio_toggoling.d \
./Src/toggoling_with_sw.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -c -I../Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/gpio_toggoling.d ./Src/gpio_toggoling.o ./Src/gpio_toggoling.su ./Src/toggoling_with_sw.d ./Src/toggoling_with_sw.o ./Src/toggoling_with_sw.su

.PHONY: clean-Src


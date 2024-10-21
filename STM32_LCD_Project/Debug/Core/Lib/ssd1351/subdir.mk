################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Lib/ssd1351/fonts.c \
../Core/Lib/ssd1351/ssd1351.c 

OBJS += \
./Core/Lib/ssd1351/fonts.o \
./Core/Lib/ssd1351/ssd1351.o 

C_DEPS += \
./Core/Lib/ssd1351/fonts.d \
./Core/Lib/ssd1351/ssd1351.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Lib/ssd1351/%.o Core/Lib/ssd1351/%.su Core/Lib/ssd1351/%.cyclo: ../Core/Lib/ssd1351/%.c Core/Lib/ssd1351/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Lib-2f-ssd1351

clean-Core-2f-Lib-2f-ssd1351:
	-$(RM) ./Core/Lib/ssd1351/fonts.cyclo ./Core/Lib/ssd1351/fonts.d ./Core/Lib/ssd1351/fonts.o ./Core/Lib/ssd1351/fonts.su ./Core/Lib/ssd1351/ssd1351.cyclo ./Core/Lib/ssd1351/ssd1351.d ./Core/Lib/ssd1351/ssd1351.o ./Core/Lib/ssd1351/ssd1351.su

.PHONY: clean-Core-2f-Lib-2f-ssd1351


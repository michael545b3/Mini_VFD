################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/adc.c \
../Core/Src/diagnosis.c \
../Core/Src/dogl128.c \
../Core/Src/eeprom.c \
../Core/Src/fonts.c \
../Core/Src/gpio.c \
../Core/Src/main.c \
../Core/Src/main2.c \
../Core/Src/menu.c \
../Core/Src/mgui.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/tca9534.c \
../Core/Src/values.c \
../Core/Src/vfd.c 

OBJS += \
./Core/Src/adc.o \
./Core/Src/diagnosis.o \
./Core/Src/dogl128.o \
./Core/Src/eeprom.o \
./Core/Src/fonts.o \
./Core/Src/gpio.o \
./Core/Src/main.o \
./Core/Src/main2.o \
./Core/Src/menu.o \
./Core/Src/mgui.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/tca9534.o \
./Core/Src/values.o \
./Core/Src/vfd.o 

C_DEPS += \
./Core/Src/adc.d \
./Core/Src/diagnosis.d \
./Core/Src/dogl128.d \
./Core/Src/eeprom.d \
./Core/Src/fonts.d \
./Core/Src/gpio.d \
./Core/Src/main.d \
./Core/Src/main2.d \
./Core/Src/menu.d \
./Core/Src/mgui.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/tca9534.d \
./Core/Src/values.d \
./Core/Src/vfd.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/diagnosis.cyclo ./Core/Src/diagnosis.d ./Core/Src/diagnosis.o ./Core/Src/diagnosis.su ./Core/Src/dogl128.cyclo ./Core/Src/dogl128.d ./Core/Src/dogl128.o ./Core/Src/dogl128.su ./Core/Src/eeprom.cyclo ./Core/Src/eeprom.d ./Core/Src/eeprom.o ./Core/Src/eeprom.su ./Core/Src/fonts.cyclo ./Core/Src/fonts.d ./Core/Src/fonts.o ./Core/Src/fonts.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/main2.cyclo ./Core/Src/main2.d ./Core/Src/main2.o ./Core/Src/main2.su ./Core/Src/menu.cyclo ./Core/Src/menu.d ./Core/Src/menu.o ./Core/Src/menu.su ./Core/Src/mgui.cyclo ./Core/Src/mgui.d ./Core/Src/mgui.o ./Core/Src/mgui.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/tca9534.cyclo ./Core/Src/tca9534.d ./Core/Src/tca9534.o ./Core/Src/tca9534.su ./Core/Src/values.cyclo ./Core/Src/values.d ./Core/Src/values.o ./Core/Src/values.su ./Core/Src/vfd.cyclo ./Core/Src/vfd.d ./Core/Src/vfd.o ./Core/Src/vfd.su

.PHONY: clean-Core-2f-Src


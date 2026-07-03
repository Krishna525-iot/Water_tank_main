################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/acs712.c \
../Core/Src/adc.c \
../Core/Src/device_id.c \
../Core/Src/eeprom_i2c.c \
../Core/Src/global.c \
../Core/Src/lcd_i2c.c \
../Core/Src/led.c \
../Core/Src/lora.c \
../Core/Src/lora_parser.c \
../Core/Src/main.c \
../Core/Src/model_handle.c \
../Core/Src/relay.c \
../Core/Src/rf.c \
../Core/Src/rtc_i2c.c \
../Core/Src/screen.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/switches.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c \
../Core/Src/uart.c \
../Core/Src/uart_commands.c 

OBJS += \
./Core/Src/acs712.o \
./Core/Src/adc.o \
./Core/Src/device_id.o \
./Core/Src/eeprom_i2c.o \
./Core/Src/global.o \
./Core/Src/lcd_i2c.o \
./Core/Src/led.o \
./Core/Src/lora.o \
./Core/Src/lora_parser.o \
./Core/Src/main.o \
./Core/Src/model_handle.o \
./Core/Src/relay.o \
./Core/Src/rf.o \
./Core/Src/rtc_i2c.o \
./Core/Src/screen.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/switches.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o \
./Core/Src/uart.o \
./Core/Src/uart_commands.o 

C_DEPS += \
./Core/Src/acs712.d \
./Core/Src/adc.d \
./Core/Src/device_id.d \
./Core/Src/eeprom_i2c.d \
./Core/Src/global.d \
./Core/Src/lcd_i2c.d \
./Core/Src/led.d \
./Core/Src/lora.d \
./Core/Src/lora_parser.d \
./Core/Src/main.d \
./Core/Src/model_handle.d \
./Core/Src/relay.d \
./Core/Src/rf.d \
./Core/Src/rtc_i2c.d \
./Core/Src/screen.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/switches.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d \
./Core/Src/uart.d \
./Core/Src/uart_commands.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -u _printf_float -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/acs712.cyclo ./Core/Src/acs712.d ./Core/Src/acs712.o ./Core/Src/acs712.su ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/device_id.cyclo ./Core/Src/device_id.d ./Core/Src/device_id.o ./Core/Src/device_id.su ./Core/Src/eeprom_i2c.cyclo ./Core/Src/eeprom_i2c.d ./Core/Src/eeprom_i2c.o ./Core/Src/eeprom_i2c.su ./Core/Src/global.cyclo ./Core/Src/global.d ./Core/Src/global.o ./Core/Src/global.su ./Core/Src/lcd_i2c.cyclo ./Core/Src/lcd_i2c.d ./Core/Src/lcd_i2c.o ./Core/Src/lcd_i2c.su ./Core/Src/led.cyclo ./Core/Src/led.d ./Core/Src/led.o ./Core/Src/led.su ./Core/Src/lora.cyclo ./Core/Src/lora.d ./Core/Src/lora.o ./Core/Src/lora.su ./Core/Src/lora_parser.cyclo ./Core/Src/lora_parser.d ./Core/Src/lora_parser.o ./Core/Src/lora_parser.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/model_handle.cyclo ./Core/Src/model_handle.d ./Core/Src/model_handle.o ./Core/Src/model_handle.su ./Core/Src/relay.cyclo ./Core/Src/relay.d ./Core/Src/relay.o ./Core/Src/relay.su ./Core/Src/rf.cyclo ./Core/Src/rf.d ./Core/Src/rf.o ./Core/Src/rf.su ./Core/Src/rtc_i2c.cyclo ./Core/Src/rtc_i2c.d ./Core/Src/rtc_i2c.o ./Core/Src/rtc_i2c.su ./Core/Src/screen.cyclo ./Core/Src/screen.d ./Core/Src/screen.o ./Core/Src/screen.su ./Core/Src/stm32f1xx_hal_msp.cyclo ./Core/Src/stm32f1xx_hal_msp.d ./Core/Src/stm32f1xx_hal_msp.o ./Core/Src/stm32f1xx_hal_msp.su ./Core/Src/stm32f1xx_it.cyclo ./Core/Src/stm32f1xx_it.d ./Core/Src/stm32f1xx_it.o ./Core/Src/stm32f1xx_it.su ./Core/Src/switches.cyclo ./Core/Src/switches.d ./Core/Src/switches.o ./Core/Src/switches.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f1xx.cyclo ./Core/Src/system_stm32f1xx.d ./Core/Src/system_stm32f1xx.o ./Core/Src/system_stm32f1xx.su ./Core/Src/uart.cyclo ./Core/Src/uart.d ./Core/Src/uart.o ./Core/Src/uart.su ./Core/Src/uart_commands.cyclo ./Core/Src/uart_commands.d ./Core/Src/uart_commands.o ./Core/Src/uart_commands.su

.PHONY: clean-Core-2f-Src


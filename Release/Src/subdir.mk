################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/gpio.c \
../Src/l3gd20.c \
../Src/lcd.c \
../Src/lsm303c.c \
../Src/main.c \
../Src/rtc.c \
../Src/spi.c \
../Src/stm32l476g_discovery.c \
../Src/stm32l476g_discovery_compass.c \
../Src/stm32l476g_discovery_glass_lcd.c \
../Src/stm32l476g_discovery_gyroscope.c \
../Src/stm32l4xx_hal_msp.c \
../Src/stm32l4xx_it.c \
../Src/system_stm32l4xx.c \
../Src/tim.c 

OBJS += \
./Src/gpio.o \
./Src/l3gd20.o \
./Src/lcd.o \
./Src/lsm303c.o \
./Src/main.o \
./Src/rtc.o \
./Src/spi.o \
./Src/stm32l476g_discovery.o \
./Src/stm32l476g_discovery_compass.o \
./Src/stm32l476g_discovery_glass_lcd.o \
./Src/stm32l476g_discovery_gyroscope.o \
./Src/stm32l4xx_hal_msp.o \
./Src/stm32l4xx_it.o \
./Src/system_stm32l4xx.o \
./Src/tim.o 

C_DEPS += \
./Src/gpio.d \
./Src/l3gd20.d \
./Src/lcd.d \
./Src/lsm303c.d \
./Src/main.d \
./Src/rtc.d \
./Src/spi.d \
./Src/stm32l476g_discovery.d \
./Src/stm32l476g_discovery_compass.d \
./Src/stm32l476g_discovery_glass_lcd.d \
./Src/stm32l476g_discovery_gyroscope.d \
./Src/stm32l4xx_hal_msp.d \
./Src/stm32l4xx_it.d \
./Src/system_stm32l4xx.d \
./Src/tim.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32L476xx -I"C:/Users/docp/Desktop/workspace/acc/Inc" -I"C:/Users/docp/Desktop/workspace/acc/Drivers/STM32L4xx_HAL_Driver/Inc" -I"C:/Users/docp/Desktop/workspace/acc/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/docp/Desktop/workspace/acc/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"C:/Users/docp/Desktop/workspace/acc/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



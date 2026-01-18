################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Laser.c \
../Core/Src/Radar.c \
../Core/Src/botones.c \
../Core/Src/ili9341.c \
../Core/Src/lcd.c \
../Core/Src/liquidcrystal_i2c.c \
../Core/Src/main.c \
../Core/Src/mapa.c \
../Core/Src/menus.c \
../Core/Src/posicion_pool.c \
../Core/Src/sensor_distancia.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/Laser.o \
./Core/Src/Radar.o \
./Core/Src/botones.o \
./Core/Src/ili9341.o \
./Core/Src/lcd.o \
./Core/Src/liquidcrystal_i2c.o \
./Core/Src/main.o \
./Core/Src/mapa.o \
./Core/Src/menus.o \
./Core/Src/posicion_pool.o \
./Core/Src/sensor_distancia.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/Laser.d \
./Core/Src/Radar.d \
./Core/Src/botones.d \
./Core/Src/ili9341.d \
./Core/Src/lcd.d \
./Core/Src/liquidcrystal_i2c.d \
./Core/Src/main.d \
./Core/Src/mapa.d \
./Core/Src/menus.d \
./Core/Src/posicion_pool.d \
./Core/Src/sensor_distancia.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I"C:/Users/ainar/OneDrive/Escritorio/trabajo_micro/Microcontrollers-project/Trabajo/Drivers/VL53L0X/core/inc" -I"C:/Users/ainar/OneDrive/Escritorio/trabajo_micro/Microcontrollers-project/Trabajo/Drivers/VL53L0X/platform/inc" -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Laser.cyclo ./Core/Src/Laser.d ./Core/Src/Laser.o ./Core/Src/Laser.su ./Core/Src/Radar.cyclo ./Core/Src/Radar.d ./Core/Src/Radar.o ./Core/Src/Radar.su ./Core/Src/botones.cyclo ./Core/Src/botones.d ./Core/Src/botones.o ./Core/Src/botones.su ./Core/Src/ili9341.cyclo ./Core/Src/ili9341.d ./Core/Src/ili9341.o ./Core/Src/ili9341.su ./Core/Src/lcd.cyclo ./Core/Src/lcd.d ./Core/Src/lcd.o ./Core/Src/lcd.su ./Core/Src/liquidcrystal_i2c.cyclo ./Core/Src/liquidcrystal_i2c.d ./Core/Src/liquidcrystal_i2c.o ./Core/Src/liquidcrystal_i2c.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mapa.cyclo ./Core/Src/mapa.d ./Core/Src/mapa.o ./Core/Src/mapa.su ./Core/Src/menus.cyclo ./Core/Src/menus.d ./Core/Src/menus.o ./Core/Src/menus.su ./Core/Src/posicion_pool.cyclo ./Core/Src/posicion_pool.d ./Core/Src/posicion_pool.o ./Core/Src/posicion_pool.su ./Core/Src/sensor_distancia.cyclo ./Core/Src/sensor_distancia.d ./Core/Src/sensor_distancia.o ./Core/Src/sensor_distancia.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src


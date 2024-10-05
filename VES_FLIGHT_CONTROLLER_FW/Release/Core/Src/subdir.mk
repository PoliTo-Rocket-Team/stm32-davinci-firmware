################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/W25Q128.c \
../Core/Src/bmp3.c \
../Core/Src/buzzer.c \
../Core/Src/codegen_model.c \
../Core/Src/codegen_model_data.c \
../Core/Src/e220.c \
../Core/Src/flight_control.c \
../Core/Src/freertos.c \
../Core/Src/lsm6dso32_reg.c \
../Core/Src/main.c \
../Core/Src/pitot_sensor.c \
../Core/Src/servo.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_hal_timebase_tim.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/test.c \
../Core/Src/utilities.c \
../Core/Src/z_flash_W25QXXX.c 

OBJS += \
./Core/Src/W25Q128.o \
./Core/Src/bmp3.o \
./Core/Src/buzzer.o \
./Core/Src/codegen_model.o \
./Core/Src/codegen_model_data.o \
./Core/Src/e220.o \
./Core/Src/flight_control.o \
./Core/Src/freertos.o \
./Core/Src/lsm6dso32_reg.o \
./Core/Src/main.o \
./Core/Src/pitot_sensor.o \
./Core/Src/servo.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_hal_timebase_tim.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/test.o \
./Core/Src/utilities.o \
./Core/Src/z_flash_W25QXXX.o 

C_DEPS += \
./Core/Src/W25Q128.d \
./Core/Src/bmp3.d \
./Core/Src/buzzer.d \
./Core/Src/codegen_model.d \
./Core/Src/codegen_model_data.d \
./Core/Src/e220.d \
./Core/Src/flight_control.d \
./Core/Src/freertos.d \
./Core/Src/lsm6dso32_reg.d \
./Core/Src/main.d \
./Core/Src/pitot_sensor.d \
./Core/Src/servo.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_hal_timebase_tim.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/test.d \
./Core/Src/utilities.d \
./Core/Src/z_flash_W25QXXX.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/W25Q128.cyclo ./Core/Src/W25Q128.d ./Core/Src/W25Q128.o ./Core/Src/W25Q128.su ./Core/Src/bmp3.cyclo ./Core/Src/bmp3.d ./Core/Src/bmp3.o ./Core/Src/bmp3.su ./Core/Src/buzzer.cyclo ./Core/Src/buzzer.d ./Core/Src/buzzer.o ./Core/Src/buzzer.su ./Core/Src/codegen_model.cyclo ./Core/Src/codegen_model.d ./Core/Src/codegen_model.o ./Core/Src/codegen_model.su ./Core/Src/codegen_model_data.cyclo ./Core/Src/codegen_model_data.d ./Core/Src/codegen_model_data.o ./Core/Src/codegen_model_data.su ./Core/Src/e220.cyclo ./Core/Src/e220.d ./Core/Src/e220.o ./Core/Src/e220.su ./Core/Src/flight_control.cyclo ./Core/Src/flight_control.d ./Core/Src/flight_control.o ./Core/Src/flight_control.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/lsm6dso32_reg.cyclo ./Core/Src/lsm6dso32_reg.d ./Core/Src/lsm6dso32_reg.o ./Core/Src/lsm6dso32_reg.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/pitot_sensor.cyclo ./Core/Src/pitot_sensor.d ./Core/Src/pitot_sensor.o ./Core/Src/pitot_sensor.su ./Core/Src/servo.cyclo ./Core/Src/servo.d ./Core/Src/servo.o ./Core/Src/servo.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_hal_timebase_tim.cyclo ./Core/Src/stm32f4xx_hal_timebase_tim.d ./Core/Src/stm32f4xx_hal_timebase_tim.o ./Core/Src/stm32f4xx_hal_timebase_tim.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/test.cyclo ./Core/Src/test.d ./Core/Src/test.o ./Core/Src/test.su ./Core/Src/utilities.cyclo ./Core/Src/utilities.d ./Core/Src/utilities.o ./Core/Src/utilities.su ./Core/Src/z_flash_W25QXXX.cyclo ./Core/Src/z_flash_W25QXXX.d ./Core/Src/z_flash_W25QXXX.o ./Core/Src/z_flash_W25QXXX.su

.PHONY: clean-Core-2f-Src


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/sensors_drivers/LSM6DSO32_test.c 

OBJS += \
./Core/sensors_drivers/LSM6DSO32_test.o 

C_DEPS += \
./Core/sensors_drivers/LSM6DSO32_test.d 


# Each subdirectory must supply rules for building sources it contributes
Core/sensors_drivers/%.o Core/sensors_drivers/%.su Core/sensors_drivers/%.cyclo: ../Core/sensors_drivers/%.c Core/sensors_drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-sensors_drivers

clean-Core-2f-sensors_drivers:
	-$(RM) ./Core/sensors_drivers/LSM6DSO32_test.cyclo ./Core/sensors_drivers/LSM6DSO32_test.d ./Core/sensors_drivers/LSM6DSO32_test.o ./Core/sensors_drivers/LSM6DSO32_test.su

.PHONY: clean-Core-2f-sensors_drivers


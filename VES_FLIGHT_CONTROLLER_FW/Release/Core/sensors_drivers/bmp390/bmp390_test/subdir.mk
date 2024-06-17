################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/sensors_drivers/bmp390/bmp390_test/bmp3.c \
../Core/sensors_drivers/bmp390/bmp390_test/usr_bmp390_task.c \
../Core/sensors_drivers/bmp390/bmp390_test/usr_common_porting.c 

OBJS += \
./Core/sensors_drivers/bmp390/bmp390_test/bmp3.o \
./Core/sensors_drivers/bmp390/bmp390_test/usr_bmp390_task.o \
./Core/sensors_drivers/bmp390/bmp390_test/usr_common_porting.o 

C_DEPS += \
./Core/sensors_drivers/bmp390/bmp390_test/bmp3.d \
./Core/sensors_drivers/bmp390/bmp390_test/usr_bmp390_task.d \
./Core/sensors_drivers/bmp390/bmp390_test/usr_common_porting.d 


# Each subdirectory must supply rules for building sources it contributes
Core/sensors_drivers/bmp390/bmp390_test/%.o Core/sensors_drivers/bmp390/bmp390_test/%.su Core/sensors_drivers/bmp390/bmp390_test/%.cyclo: ../Core/sensors_drivers/bmp390/bmp390_test/%.c Core/sensors_drivers/bmp390/bmp390_test/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-sensors_drivers-2f-bmp390-2f-bmp390_test

clean-Core-2f-sensors_drivers-2f-bmp390-2f-bmp390_test:
	-$(RM) ./Core/sensors_drivers/bmp390/bmp390_test/bmp3.cyclo ./Core/sensors_drivers/bmp390/bmp390_test/bmp3.d ./Core/sensors_drivers/bmp390/bmp390_test/bmp3.o ./Core/sensors_drivers/bmp390/bmp390_test/bmp3.su ./Core/sensors_drivers/bmp390/bmp390_test/usr_bmp390_task.cyclo ./Core/sensors_drivers/bmp390/bmp390_test/usr_bmp390_task.d ./Core/sensors_drivers/bmp390/bmp390_test/usr_bmp390_task.o ./Core/sensors_drivers/bmp390/bmp390_test/usr_bmp390_task.su ./Core/sensors_drivers/bmp390/bmp390_test/usr_common_porting.cyclo ./Core/sensors_drivers/bmp390/bmp390_test/usr_common_porting.d ./Core/sensors_drivers/bmp390/bmp390_test/usr_common_porting.o ./Core/sensors_drivers/bmp390/bmp390_test/usr_common_porting.su

.PHONY: clean-Core-2f-sensors_drivers-2f-bmp390-2f-bmp390_test


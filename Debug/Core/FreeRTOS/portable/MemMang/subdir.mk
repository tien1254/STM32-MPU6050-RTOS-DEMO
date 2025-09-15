################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/FreeRTOS/portable/MemMang/heap_4.c 

OBJS += \
./Core/FreeRTOS/portable/MemMang/heap_4.o 

C_DEPS += \
./Core/FreeRTOS/portable/MemMang/heap_4.d 


# Each subdirectory must supply rules for building sources it contributes
Core/FreeRTOS/portable/MemMang/%.o Core/FreeRTOS/portable/MemMang/%.su Core/FreeRTOS/portable/MemMang/%.cyclo: ../Core/FreeRTOS/portable/MemMang/%.c Core/FreeRTOS/portable/MemMang/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS/include" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS/portable" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS/portable/GCC/ARM_CM3" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS/portable/MemMang" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/SeggerSystemView/Config" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/SeggerSystemView/FreeRTOSV11" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/SeggerSystemView/SEGGER" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/SeggerSystemView/SEGGER/Syscalls" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS/portable/GCC" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS/SEGGER" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS/SEGGER/Config" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS/SEGGER/OS" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS/SEGGER/Patch" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS/SEGGER/SEGGER" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-FreeRTOS-2f-portable-2f-MemMang

clean-Core-2f-FreeRTOS-2f-portable-2f-MemMang:
	-$(RM) ./Core/FreeRTOS/portable/MemMang/heap_4.cyclo ./Core/FreeRTOS/portable/MemMang/heap_4.d ./Core/FreeRTOS/portable/MemMang/heap_4.o ./Core/FreeRTOS/portable/MemMang/heap_4.su

.PHONY: clean-Core-2f-FreeRTOS-2f-portable-2f-MemMang


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/FreeRTOS/croutine.c \
../Core/FreeRTOS/event_groups.c \
../Core/FreeRTOS/list.c \
../Core/FreeRTOS/queue.c \
../Core/FreeRTOS/stream_buffer.c \
../Core/FreeRTOS/tasks.c \
../Core/FreeRTOS/timers.c 

OBJS += \
./Core/FreeRTOS/croutine.o \
./Core/FreeRTOS/event_groups.o \
./Core/FreeRTOS/list.o \
./Core/FreeRTOS/queue.o \
./Core/FreeRTOS/stream_buffer.o \
./Core/FreeRTOS/tasks.o \
./Core/FreeRTOS/timers.o 

C_DEPS += \
./Core/FreeRTOS/croutine.d \
./Core/FreeRTOS/event_groups.d \
./Core/FreeRTOS/list.d \
./Core/FreeRTOS/queue.d \
./Core/FreeRTOS/stream_buffer.d \
./Core/FreeRTOS/tasks.d \
./Core/FreeRTOS/timers.d 


# Each subdirectory must supply rules for building sources it contributes
Core/FreeRTOS/%.o Core/FreeRTOS/%.su Core/FreeRTOS/%.cyclo: ../Core/FreeRTOS/%.c Core/FreeRTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS/include" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS/portable" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS/portable/GCC/ARM_CM3" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS/portable/MemMang" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/SeggerSystemView/Config" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/SeggerSystemView/FreeRTOSV11" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/SeggerSystemView/SEGGER" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/SeggerSystemView/SEGGER/Syscalls" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS/portable/GCC" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS/SEGGER" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS/SEGGER/Config" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS/SEGGER/OS" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS/SEGGER/Patch" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS/SEGGER/SEGGER" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-FreeRTOS

clean-Core-2f-FreeRTOS:
	-$(RM) ./Core/FreeRTOS/croutine.cyclo ./Core/FreeRTOS/croutine.d ./Core/FreeRTOS/croutine.o ./Core/FreeRTOS/croutine.su ./Core/FreeRTOS/event_groups.cyclo ./Core/FreeRTOS/event_groups.d ./Core/FreeRTOS/event_groups.o ./Core/FreeRTOS/event_groups.su ./Core/FreeRTOS/list.cyclo ./Core/FreeRTOS/list.d ./Core/FreeRTOS/list.o ./Core/FreeRTOS/list.su ./Core/FreeRTOS/queue.cyclo ./Core/FreeRTOS/queue.d ./Core/FreeRTOS/queue.o ./Core/FreeRTOS/queue.su ./Core/FreeRTOS/stream_buffer.cyclo ./Core/FreeRTOS/stream_buffer.d ./Core/FreeRTOS/stream_buffer.o ./Core/FreeRTOS/stream_buffer.su ./Core/FreeRTOS/tasks.cyclo ./Core/FreeRTOS/tasks.d ./Core/FreeRTOS/tasks.o ./Core/FreeRTOS/tasks.su ./Core/FreeRTOS/timers.cyclo ./Core/FreeRTOS/timers.d ./Core/FreeRTOS/timers.o ./Core/FreeRTOS/timers.su

.PHONY: clean-Core-2f-FreeRTOS


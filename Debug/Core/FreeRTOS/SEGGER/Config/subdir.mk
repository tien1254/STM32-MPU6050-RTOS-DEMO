################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/FreeRTOS/SEGGER/Config/SEGGER_SYSVIEW_Config_FreeRTOS.c 

OBJS += \
./Core/FreeRTOS/SEGGER/Config/SEGGER_SYSVIEW_Config_FreeRTOS.o 

C_DEPS += \
./Core/FreeRTOS/SEGGER/Config/SEGGER_SYSVIEW_Config_FreeRTOS.d 


# Each subdirectory must supply rules for building sources it contributes
Core/FreeRTOS/SEGGER/Config/%.o Core/FreeRTOS/SEGGER/Config/%.su Core/FreeRTOS/SEGGER/Config/%.cyclo: ../Core/FreeRTOS/SEGGER/Config/%.c Core/FreeRTOS/SEGGER/Config/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS/include" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS/portable" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS/portable/GCC/ARM_CM3" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS/portable/MemMang" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/SeggerSystemView/Config" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/SeggerSystemView/FreeRTOSV11" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/SeggerSystemView/SEGGER" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/SeggerSystemView/SEGGER/Syscalls" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS/portable/GCC" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS/SEGGER" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS/SEGGER/Config" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS/SEGGER/OS" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS/SEGGER/Patch" -I"D:/2_STM32/bai_tap_vv/12_MPU6050/Core/FreeRTOS/SEGGER/SEGGER" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-FreeRTOS-2f-SEGGER-2f-Config

clean-Core-2f-FreeRTOS-2f-SEGGER-2f-Config:
	-$(RM) ./Core/FreeRTOS/SEGGER/Config/SEGGER_SYSVIEW_Config_FreeRTOS.cyclo ./Core/FreeRTOS/SEGGER/Config/SEGGER_SYSVIEW_Config_FreeRTOS.d ./Core/FreeRTOS/SEGGER/Config/SEGGER_SYSVIEW_Config_FreeRTOS.o ./Core/FreeRTOS/SEGGER/Config/SEGGER_SYSVIEW_Config_FreeRTOS.su

.PHONY: clean-Core-2f-FreeRTOS-2f-SEGGER-2f-Config


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../L4_IOT_Sensors/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.c \
../L4_IOT_Sensors/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.c 

OBJS += \
./L4_IOT_Sensors/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.o \
./L4_IOT_Sensors/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.o 

C_DEPS += \
./L4_IOT_Sensors/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.d \
./L4_IOT_Sensors/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.d 


# Each subdirectory must supply rules for building sources it contributes
L4_IOT_Sensors/Drivers/BSP/B-L475E-IOT01/%.o L4_IOT_Sensors/Drivers/BSP/B-L475E-IOT01/%.su: ../L4_IOT_Sensors/Drivers/BSP/B-L475E-IOT01/%.c L4_IOT_Sensors/Drivers/BSP/B-L475E-IOT01/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I"C:/Users/anton/STM32CubeIDE/trabajoSCSH/Practica5/Libraries/WIFI/Common/Inc" -I"C:/Users/anton/STM32CubeIDE/trabajoSCSH/Practica5/L4_IOT_Sensors/Drivers/BSP/B-L475E-IOT01" -I"C:/Users/anton/STM32CubeIDE/trabajoSCSH/Practica5/L4_IOT_Sensors/Drivers/BSP/Components/lsm6dsl" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-L4_IOT_Sensors-2f-Drivers-2f-BSP-2f-B-2d-L475E-2d-IOT01

clean-L4_IOT_Sensors-2f-Drivers-2f-BSP-2f-B-2d-L475E-2d-IOT01:
	-$(RM) ./L4_IOT_Sensors/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.d ./L4_IOT_Sensors/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.o ./L4_IOT_Sensors/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.su ./L4_IOT_Sensors/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.d ./L4_IOT_Sensors/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.o ./L4_IOT_Sensors/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.su

.PHONY: clean-L4_IOT_Sensors-2f-Drivers-2f-BSP-2f-B-2d-L475E-2d-IOT01


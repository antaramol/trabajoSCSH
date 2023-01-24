################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/coreMQTT/source/interface/transport_interface.c 

OBJS += \
./Libraries/coreMQTT/source/interface/transport_interface.o 

C_DEPS += \
./Libraries/coreMQTT/source/interface/transport_interface.d 


# Each subdirectory must supply rules for building sources it contributes
Libraries/coreMQTT/source/interface/%.o Libraries/coreMQTT/source/interface/%.su: ../Libraries/coreMQTT/source/interface/%.c Libraries/coreMQTT/source/interface/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I"C:/Users/Jacin/workspace/trabajoSCSH/Practica5/Libraries/coreMQTT/source/include" -I"C:/Users/Jacin/workspace/trabajoSCSH/Practica5/Libraries/coreMQTT/source/interface" -I"C:/Users/Jacin/workspace/trabajoSCSH/Practica5/Libraries/WIFI/Common/Inc" -I"C:/Users/Jacin/workspace/trabajoSCSH/Practica5/L4_IOT_Sensors/Drivers/BSP/B-L475E-IOT01" -I"C:/Users/Jacin/workspace/trabajoSCSH/Practica5/L4_IOT_Sensors/Drivers/BSP/Components/lsm6dsl" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Libraries-2f-coreMQTT-2f-source-2f-interface

clean-Libraries-2f-coreMQTT-2f-source-2f-interface:
	-$(RM) ./Libraries/coreMQTT/source/interface/transport_interface.d ./Libraries/coreMQTT/source/interface/transport_interface.o ./Libraries/coreMQTT/source/interface/transport_interface.su

.PHONY: clean-Libraries-2f-coreMQTT-2f-source-2f-interface


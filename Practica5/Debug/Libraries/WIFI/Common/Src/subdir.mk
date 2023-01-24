################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/WIFI/Common/Src/es_wifi.c \
../Libraries/WIFI/Common/Src/es_wifi_io.c \
../Libraries/WIFI/Common/Src/wifi.c 

OBJS += \
./Libraries/WIFI/Common/Src/es_wifi.o \
./Libraries/WIFI/Common/Src/es_wifi_io.o \
./Libraries/WIFI/Common/Src/wifi.o 

C_DEPS += \
./Libraries/WIFI/Common/Src/es_wifi.d \
./Libraries/WIFI/Common/Src/es_wifi_io.d \
./Libraries/WIFI/Common/Src/wifi.d 


# Each subdirectory must supply rules for building sources it contributes
Libraries/WIFI/Common/Src/%.o Libraries/WIFI/Common/Src/%.su: ../Libraries/WIFI/Common/Src/%.c Libraries/WIFI/Common/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I"C:/Users/Jacin/workspace/trabajoSCSH/Practica5/Libraries/coreMQTT/source/include" -I"C:/Users/Jacin/workspace/trabajoSCSH/Practica5/Libraries/coreMQTT/source/interface" -I"C:/Users/Jacin/workspace/trabajoSCSH/Practica5/Libraries/WIFI/Common/Inc" -I"C:/Users/Jacin/workspace/trabajoSCSH/Practica5/L4_IOT_Sensors/Drivers/BSP/B-L475E-IOT01" -I"C:/Users/Jacin/workspace/trabajoSCSH/Practica5/L4_IOT_Sensors/Drivers/BSP/Components/lsm6dsl" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Libraries-2f-WIFI-2f-Common-2f-Src

clean-Libraries-2f-WIFI-2f-Common-2f-Src:
	-$(RM) ./Libraries/WIFI/Common/Src/es_wifi.d ./Libraries/WIFI/Common/Src/es_wifi.o ./Libraries/WIFI/Common/Src/es_wifi.su ./Libraries/WIFI/Common/Src/es_wifi_io.d ./Libraries/WIFI/Common/Src/es_wifi_io.o ./Libraries/WIFI/Common/Src/es_wifi_io.su ./Libraries/WIFI/Common/Src/wifi.d ./Libraries/WIFI/Common/Src/wifi.o ./Libraries/WIFI/Common/Src/wifi.su

.PHONY: clean-Libraries-2f-WIFI-2f-Common-2f-Src


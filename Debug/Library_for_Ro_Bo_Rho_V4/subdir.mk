################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Library_for_Ro_Bo_Rho_V4/Arduino.c \
../Library_for_Ro_Bo_Rho_V4/Encoder_Motor.c \
../Library_for_Ro_Bo_Rho_V4/Forward_Kinematics.c \
../Library_for_Ro_Bo_Rho_V4/Game_Play.c \
../Library_for_Ro_Bo_Rho_V4/Inverse_Kinematics.c \
../Library_for_Ro_Bo_Rho_V4/MPU6050.c \
../Library_for_Ro_Bo_Rho_V4/Ramp_Robot.c \
../Library_for_Ro_Bo_Rho_V4/UART_ESP.c 

OBJS += \
./Library_for_Ro_Bo_Rho_V4/Arduino.o \
./Library_for_Ro_Bo_Rho_V4/Encoder_Motor.o \
./Library_for_Ro_Bo_Rho_V4/Forward_Kinematics.o \
./Library_for_Ro_Bo_Rho_V4/Game_Play.o \
./Library_for_Ro_Bo_Rho_V4/Inverse_Kinematics.o \
./Library_for_Ro_Bo_Rho_V4/MPU6050.o \
./Library_for_Ro_Bo_Rho_V4/Ramp_Robot.o \
./Library_for_Ro_Bo_Rho_V4/UART_ESP.o 

C_DEPS += \
./Library_for_Ro_Bo_Rho_V4/Arduino.d \
./Library_for_Ro_Bo_Rho_V4/Encoder_Motor.d \
./Library_for_Ro_Bo_Rho_V4/Forward_Kinematics.d \
./Library_for_Ro_Bo_Rho_V4/Game_Play.d \
./Library_for_Ro_Bo_Rho_V4/Inverse_Kinematics.d \
./Library_for_Ro_Bo_Rho_V4/MPU6050.d \
./Library_for_Ro_Bo_Rho_V4/Ramp_Robot.d \
./Library_for_Ro_Bo_Rho_V4/UART_ESP.d 


# Each subdirectory must supply rules for building sources it contributes
Library_for_Ro_Bo_Rho_V4/%.o Library_for_Ro_Bo_Rho_V4/%.su Library_for_Ro_Bo_Rho_V4/%.cyclo: ../Library_for_Ro_Bo_Rho_V4/%.c Library_for_Ro_Bo_Rho_V4/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Knnn/STM32CubeIDE/workspace_1.15.1/Ro_Bo_Rho_V4/Library_for_Ro_Bo_Rho_V4" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Library_for_Ro_Bo_Rho_V4

clean-Library_for_Ro_Bo_Rho_V4:
	-$(RM) ./Library_for_Ro_Bo_Rho_V4/Arduino.cyclo ./Library_for_Ro_Bo_Rho_V4/Arduino.d ./Library_for_Ro_Bo_Rho_V4/Arduino.o ./Library_for_Ro_Bo_Rho_V4/Arduino.su ./Library_for_Ro_Bo_Rho_V4/Encoder_Motor.cyclo ./Library_for_Ro_Bo_Rho_V4/Encoder_Motor.d ./Library_for_Ro_Bo_Rho_V4/Encoder_Motor.o ./Library_for_Ro_Bo_Rho_V4/Encoder_Motor.su ./Library_for_Ro_Bo_Rho_V4/Forward_Kinematics.cyclo ./Library_for_Ro_Bo_Rho_V4/Forward_Kinematics.d ./Library_for_Ro_Bo_Rho_V4/Forward_Kinematics.o ./Library_for_Ro_Bo_Rho_V4/Forward_Kinematics.su ./Library_for_Ro_Bo_Rho_V4/Game_Play.cyclo ./Library_for_Ro_Bo_Rho_V4/Game_Play.d ./Library_for_Ro_Bo_Rho_V4/Game_Play.o ./Library_for_Ro_Bo_Rho_V4/Game_Play.su ./Library_for_Ro_Bo_Rho_V4/Inverse_Kinematics.cyclo ./Library_for_Ro_Bo_Rho_V4/Inverse_Kinematics.d ./Library_for_Ro_Bo_Rho_V4/Inverse_Kinematics.o ./Library_for_Ro_Bo_Rho_V4/Inverse_Kinematics.su ./Library_for_Ro_Bo_Rho_V4/MPU6050.cyclo ./Library_for_Ro_Bo_Rho_V4/MPU6050.d ./Library_for_Ro_Bo_Rho_V4/MPU6050.o ./Library_for_Ro_Bo_Rho_V4/MPU6050.su ./Library_for_Ro_Bo_Rho_V4/Ramp_Robot.cyclo ./Library_for_Ro_Bo_Rho_V4/Ramp_Robot.d ./Library_for_Ro_Bo_Rho_V4/Ramp_Robot.o ./Library_for_Ro_Bo_Rho_V4/Ramp_Robot.su ./Library_for_Ro_Bo_Rho_V4/UART_ESP.cyclo ./Library_for_Ro_Bo_Rho_V4/UART_ESP.d ./Library_for_Ro_Bo_Rho_V4/UART_ESP.o ./Library_for_Ro_Bo_Rho_V4/UART_ESP.su

.PHONY: clean-Library_for_Ro_Bo_Rho_V4


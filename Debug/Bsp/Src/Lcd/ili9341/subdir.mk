################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Bsp/Src/Lcd/ili9341/ili9341hal.c 

C_DEPS += \
./Bsp/Src/Lcd/ili9341/ili9341hal.d 

OBJS += \
./Bsp/Src/Lcd/ili9341/ili9341hal.o 


# Each subdirectory must supply rules for building sources it contributes
Bsp/Src/Lcd/ili9341/%.o Bsp/Src/Lcd/ili9341/%.su Bsp/Src/Lcd/ili9341/%.cyclo: ../Bsp/Src/Lcd/ili9341/%.c Bsp/Src/Lcd/ili9341/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F413xx -DUSE_HAL_LOGGING -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Bsp/Inc/OS -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Bsp/Inc -I../Bsp/Src/Lcd -I../Bsp/Inc/IMU -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Bsp-2f-Src-2f-Lcd-2f-ili9341

clean-Bsp-2f-Src-2f-Lcd-2f-ili9341:
	-$(RM) ./Bsp/Src/Lcd/ili9341/ili9341hal.cyclo ./Bsp/Src/Lcd/ili9341/ili9341hal.d ./Bsp/Src/Lcd/ili9341/ili9341hal.o ./Bsp/Src/Lcd/ili9341/ili9341hal.su

.PHONY: clean-Bsp-2f-Src-2f-Lcd-2f-ili9341


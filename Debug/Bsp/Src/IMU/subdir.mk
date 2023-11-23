################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Bsp/Src/IMU/BMI160.cpp 

OBJS += \
./Bsp/Src/IMU/BMI160.o 

CPP_DEPS += \
./Bsp/Src/IMU/BMI160.d 


# Each subdirectory must supply rules for building sources it contributes
Bsp/Src/IMU/%.o Bsp/Src/IMU/%.su Bsp/Src/IMU/%.cyclo: ../Bsp/Src/IMU/%.cpp Bsp/Src/IMU/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F413xx -DUSE_HAL_LOGGING -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Bsp/Inc/OS -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Bsp/Inc -I../Bsp/Src/Lcd -I../Bsp/Inc/IMU -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Bsp-2f-Src-2f-IMU

clean-Bsp-2f-Src-2f-IMU:
	-$(RM) ./Bsp/Src/IMU/BMI160.cyclo ./Bsp/Src/IMU/BMI160.d ./Bsp/Src/IMU/BMI160.o ./Bsp/Src/IMU/BMI160.su

.PHONY: clean-Bsp-2f-Src-2f-IMU


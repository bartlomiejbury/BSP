################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Bsp/Src/IMUManager/BMI160.cpp \
../Bsp/Src/IMUManager/BMI160_Gravity.cpp \
../Bsp/Src/IMUManager/DFRobot_BMM150.cpp 

OBJS += \
./Bsp/Src/IMUManager/BMI160.o \
./Bsp/Src/IMUManager/BMI160_Gravity.o \
./Bsp/Src/IMUManager/DFRobot_BMM150.o 

CPP_DEPS += \
./Bsp/Src/IMUManager/BMI160.d \
./Bsp/Src/IMUManager/BMI160_Gravity.d \
./Bsp/Src/IMUManager/DFRobot_BMM150.d 


# Each subdirectory must supply rules for building sources it contributes
Bsp/Src/IMUManager/%.o Bsp/Src/IMUManager/%.su Bsp/Src/IMUManager/%.cyclo: ../Bsp/Src/IMUManager/%.cpp Bsp/Src/IMUManager/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F413xx -DUSE_HAL_LOGGING -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Bsp/Inc/ButtonManager -I../Bsp/Inc/OS -I../Bsp/Inc/IMUManager -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Bsp/Inc -I../Bsp/Src/Lcd -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Bsp-2f-Src-2f-IMUManager

clean-Bsp-2f-Src-2f-IMUManager:
	-$(RM) ./Bsp/Src/IMUManager/BMI160.cyclo ./Bsp/Src/IMUManager/BMI160.d ./Bsp/Src/IMUManager/BMI160.o ./Bsp/Src/IMUManager/BMI160.su ./Bsp/Src/IMUManager/BMI160_Gravity.cyclo ./Bsp/Src/IMUManager/BMI160_Gravity.d ./Bsp/Src/IMUManager/BMI160_Gravity.o ./Bsp/Src/IMUManager/BMI160_Gravity.su ./Bsp/Src/IMUManager/DFRobot_BMM150.cyclo ./Bsp/Src/IMUManager/DFRobot_BMM150.d ./Bsp/Src/IMUManager/DFRobot_BMM150.o ./Bsp/Src/IMUManager/DFRobot_BMM150.su

.PHONY: clean-Bsp-2f-Src-2f-IMUManager


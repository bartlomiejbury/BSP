################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Bsp/Src/ButtonManager/ButtonManager.cpp \
../Bsp/Src/ButtonManager/NaviButtonImpl.cpp 

OBJS += \
./Bsp/Src/ButtonManager/ButtonManager.o \
./Bsp/Src/ButtonManager/NaviButtonImpl.o 

CPP_DEPS += \
./Bsp/Src/ButtonManager/ButtonManager.d \
./Bsp/Src/ButtonManager/NaviButtonImpl.d 


# Each subdirectory must supply rules for building sources it contributes
Bsp/Src/ButtonManager/%.o Bsp/Src/ButtonManager/%.su Bsp/Src/ButtonManager/%.cyclo: ../Bsp/Src/ButtonManager/%.cpp Bsp/Src/ButtonManager/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F413xx -DUSE_HAL_LOGGING -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Bsp/Inc/ButtonManager -I../Bsp/Inc/OS -I../Bsp/Inc/IMUManager -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Bsp/Inc -I../Bsp/Src/Lcd -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Bsp-2f-Src-2f-ButtonManager

clean-Bsp-2f-Src-2f-ButtonManager:
	-$(RM) ./Bsp/Src/ButtonManager/ButtonManager.cyclo ./Bsp/Src/ButtonManager/ButtonManager.d ./Bsp/Src/ButtonManager/ButtonManager.o ./Bsp/Src/ButtonManager/ButtonManager.su ./Bsp/Src/ButtonManager/NaviButtonImpl.cyclo ./Bsp/Src/ButtonManager/NaviButtonImpl.d ./Bsp/Src/ButtonManager/NaviButtonImpl.o ./Bsp/Src/ButtonManager/NaviButtonImpl.su

.PHONY: clean-Bsp-2f-Src-2f-ButtonManager


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../evertCore/hrtimhandler_dcdc.c 

OBJS += \
./evertCore/hrtimhandler_dcdc.o 

C_DEPS += \
./evertCore/hrtimhandler_dcdc.d 


# Each subdirectory must supply rules for building sources it contributes
evertCore/%.o evertCore/%.su evertCore/%.cyclo: ../evertCore/%.c evertCore/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/JavierMunozSaez/Documents/GitHub/getting-started-HRTIM-phase-shift-nucleoG4/getting-started-HRTIM-nucleog4/evertCore" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-evertCore

clean-evertCore:
	-$(RM) ./evertCore/hrtimhandler_dcdc.cyclo ./evertCore/hrtimhandler_dcdc.d ./evertCore/hrtimhandler_dcdc.o ./evertCore/hrtimhandler_dcdc.su

.PHONY: clean-evertCore


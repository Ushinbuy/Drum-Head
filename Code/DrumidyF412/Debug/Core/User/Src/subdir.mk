################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/User/Src/drumidy.c \
../Core/User/Src/midi.c \
../Core/User/Src/usbd_cdc.c 

OBJS += \
./Core/User/Src/drumidy.o \
./Core/User/Src/midi.o \
./Core/User/Src/usbd_cdc.o 

C_DEPS += \
./Core/User/Src/drumidy.d \
./Core/User/Src/midi.d \
./Core/User/Src/usbd_cdc.d 


# Each subdirectory must supply rules for building sources it contributes
Core/User/Src/%.o: ../Core/User/Src/%.c Core/User/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F412Zx -c -I../FATFS/Target -I../FATFS/App -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Core/User/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-User-2f-Src

clean-Core-2f-User-2f-Src:
	-$(RM) ./Core/User/Src/drumidy.d ./Core/User/Src/drumidy.o ./Core/User/Src/midi.d ./Core/User/Src/midi.o ./Core/User/Src/usbd_cdc.d ./Core/User/Src/usbd_cdc.o

.PHONY: clean-Core-2f-User-2f-Src


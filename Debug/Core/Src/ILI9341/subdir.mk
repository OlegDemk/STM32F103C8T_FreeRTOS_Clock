################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ILI9341/ILI9341_GFX.c \
../Core/Src/ILI9341/ILI9341_STM32_Driver.c \
../Core/Src/ILI9341/ILI9341_Touchscreen.c \
../Core/Src/ILI9341/font12.c \
../Core/Src/ILI9341/font16.c \
../Core/Src/ILI9341/font20.c \
../Core/Src/ILI9341/font24.c \
../Core/Src/ILI9341/main_lcd.c \
../Core/Src/ILI9341/spi_ili9341.c 

OBJS += \
./Core/Src/ILI9341/ILI9341_GFX.o \
./Core/Src/ILI9341/ILI9341_STM32_Driver.o \
./Core/Src/ILI9341/ILI9341_Touchscreen.o \
./Core/Src/ILI9341/font12.o \
./Core/Src/ILI9341/font16.o \
./Core/Src/ILI9341/font20.o \
./Core/Src/ILI9341/font24.o \
./Core/Src/ILI9341/main_lcd.o \
./Core/Src/ILI9341/spi_ili9341.o 

C_DEPS += \
./Core/Src/ILI9341/ILI9341_GFX.d \
./Core/Src/ILI9341/ILI9341_STM32_Driver.d \
./Core/Src/ILI9341/ILI9341_Touchscreen.d \
./Core/Src/ILI9341/font12.d \
./Core/Src/ILI9341/font16.d \
./Core/Src/ILI9341/font20.d \
./Core/Src/ILI9341/font24.d \
./Core/Src/ILI9341/main_lcd.d \
./Core/Src/ILI9341/spi_ili9341.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/ILI9341/%.o Core/Src/ILI9341/%.su: ../Core/Src/ILI9341/%.c Core/Src/ILI9341/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-ILI9341

clean-Core-2f-Src-2f-ILI9341:
	-$(RM) ./Core/Src/ILI9341/ILI9341_GFX.d ./Core/Src/ILI9341/ILI9341_GFX.o ./Core/Src/ILI9341/ILI9341_GFX.su ./Core/Src/ILI9341/ILI9341_STM32_Driver.d ./Core/Src/ILI9341/ILI9341_STM32_Driver.o ./Core/Src/ILI9341/ILI9341_STM32_Driver.su ./Core/Src/ILI9341/ILI9341_Touchscreen.d ./Core/Src/ILI9341/ILI9341_Touchscreen.o ./Core/Src/ILI9341/ILI9341_Touchscreen.su ./Core/Src/ILI9341/font12.d ./Core/Src/ILI9341/font12.o ./Core/Src/ILI9341/font12.su ./Core/Src/ILI9341/font16.d ./Core/Src/ILI9341/font16.o ./Core/Src/ILI9341/font16.su ./Core/Src/ILI9341/font20.d ./Core/Src/ILI9341/font20.o ./Core/Src/ILI9341/font20.su ./Core/Src/ILI9341/font24.d ./Core/Src/ILI9341/font24.o ./Core/Src/ILI9341/font24.su ./Core/Src/ILI9341/main_lcd.d ./Core/Src/ILI9341/main_lcd.o ./Core/Src/ILI9341/main_lcd.su ./Core/Src/ILI9341/spi_ili9341.d ./Core/Src/ILI9341/spi_ili9341.o ./Core/Src/ILI9341/spi_ili9341.su

.PHONY: clean-Core-2f-Src-2f-ILI9341


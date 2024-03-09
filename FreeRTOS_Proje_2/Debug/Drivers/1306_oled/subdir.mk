################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/1306_oled/ssd1306.c \
../Drivers/1306_oled/ssd1306_fonts.c 

OBJS += \
./Drivers/1306_oled/ssd1306.o \
./Drivers/1306_oled/ssd1306_fonts.o 

C_DEPS += \
./Drivers/1306_oled/ssd1306.d \
./Drivers/1306_oled/ssd1306_fonts.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/1306_oled/%.o Drivers/1306_oled/%.su: ../Drivers/1306_oled/%.c Drivers/1306_oled/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"C:/Users/doguk/STM32CubeIDE/workspace_1.9.0/FreeRTOS_Proje_2/Drivers/1306_oled" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-1306_oled

clean-Drivers-2f-1306_oled:
	-$(RM) ./Drivers/1306_oled/ssd1306.d ./Drivers/1306_oled/ssd1306.o ./Drivers/1306_oled/ssd1306.su ./Drivers/1306_oled/ssd1306_fonts.d ./Drivers/1306_oled/ssd1306_fonts.o ./Drivers/1306_oled/ssd1306_fonts.su

.PHONY: clean-Drivers-2f-1306_oled


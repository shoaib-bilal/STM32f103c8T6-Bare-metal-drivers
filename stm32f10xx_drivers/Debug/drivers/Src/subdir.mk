################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32f103xx_i2c_driver.c \
../drivers/Src/stm32f103xx_rcc_driver.c \
../drivers/Src/stm32f103xx_spi_driver.c \
../drivers/Src/stm32f103xx_usart_driver.c \
../drivers/Src/stm32f10xx_gpio_driver.c 

OBJS += \
./drivers/Src/stm32f103xx_i2c_driver.o \
./drivers/Src/stm32f103xx_rcc_driver.o \
./drivers/Src/stm32f103xx_spi_driver.o \
./drivers/Src/stm32f103xx_usart_driver.o \
./drivers/Src/stm32f10xx_gpio_driver.o 

C_DEPS += \
./drivers/Src/stm32f103xx_i2c_driver.d \
./drivers/Src/stm32f103xx_rcc_driver.d \
./drivers/Src/stm32f103xx_spi_driver.d \
./drivers/Src/stm32f103xx_usart_driver.d \
./drivers/Src/stm32f10xx_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -c -I../Inc -I"D:/shoaib data/Embedded C/My Workspace/Target/stm32f10xx_drivers/drivers/Inc" -I"D:/shoaib data/Embedded C/My Workspace/Target/stm32f10xx_drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-Src

clean-drivers-2f-Src:
	-$(RM) ./drivers/Src/stm32f103xx_i2c_driver.d ./drivers/Src/stm32f103xx_i2c_driver.o ./drivers/Src/stm32f103xx_rcc_driver.d ./drivers/Src/stm32f103xx_rcc_driver.o ./drivers/Src/stm32f103xx_spi_driver.d ./drivers/Src/stm32f103xx_spi_driver.o ./drivers/Src/stm32f103xx_usart_driver.d ./drivers/Src/stm32f103xx_usart_driver.o ./drivers/Src/stm32f10xx_gpio_driver.d ./drivers/Src/stm32f10xx_gpio_driver.o

.PHONY: clean-drivers-2f-Src


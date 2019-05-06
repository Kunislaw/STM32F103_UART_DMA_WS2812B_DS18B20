################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/ds18b20.c \
../Src/ds18b20uart.c \
../Src/main.c \
../Src/stm32f1xx_hal_msp.c \
../Src/stm32f1xx_it.c \
../Src/system_stm32f1xx.c \
../Src/ws2812b.c 

OBJS += \
./Src/ds18b20.o \
./Src/ds18b20uart.o \
./Src/main.o \
./Src/stm32f1xx_hal_msp.o \
./Src/stm32f1xx_it.o \
./Src/system_stm32f1xx.o \
./Src/ws2812b.o 

C_DEPS += \
./Src/ds18b20.d \
./Src/ds18b20uart.d \
./Src/main.d \
./Src/stm32f1xx_hal_msp.d \
./Src/stm32f1xx_it.d \
./Src/system_stm32f1xx.d \
./Src/ws2812b.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F103xB -I"D:/OneDrive - Uniwersytet Technologiczno-Przyrodniczy/SemIII/LabMikro/ProjektMikroDom/Inc" -I"D:/OneDrive - Uniwersytet Technologiczno-Przyrodniczy/SemIII/LabMikro/ProjektMikroDom/Drivers/STM32F1xx_HAL_Driver/Inc" -I"D:/OneDrive - Uniwersytet Technologiczno-Przyrodniczy/SemIII/LabMikro/ProjektMikroDom/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"D:/OneDrive - Uniwersytet Technologiczno-Przyrodniczy/SemIII/LabMikro/ProjektMikroDom/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"D:/OneDrive - Uniwersytet Technologiczno-Przyrodniczy/SemIII/LabMikro/ProjektMikroDom/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Sources/Application.c \
../Sources/Events.c \
../Sources/Shell.c \
../Sources/main.c 

OBJS += \
./Sources/Application.o \
./Sources/Events.o \
./Sources/Shell.o \
./Sources/main.o 

C_DEPS += \
./Sources/Application.d \
./Sources/Events.d \
./Sources/Shell.d \
./Sources/main.d 


# Each subdirectory must supply rules for building sources it contributes
Sources/%.o: ../Sources/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -I"C:/Users/sword/Documents/GitHub/BatteryTestSystem/BatteryTestSystemC/Static_Code/PDD" -I"C:/Users/sword/Documents/GitHub/BatteryTestSystem/BatteryTestSystemC/Static_Code/IO_Map" -I"C:/Users/sword/Documents/GitHub/BatteryTestSystem/BatteryTestSystemC/Sources" -I"C:/Users/sword/Documents/GitHub/BatteryTestSystem/BatteryTestSystemC/Generated_Code" -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



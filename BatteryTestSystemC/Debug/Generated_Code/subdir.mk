################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Generated_Code/AD1.c \
../Generated_Code/AS1.c \
../Generated_Code/ASerialLdd1.c \
../Generated_Code/AdcLdd1.c \
../Generated_Code/BitIoLdd1.c \
../Generated_Code/BitIoLdd2.c \
../Generated_Code/CHG_PWM.c \
../Generated_Code/CLS1.c \
../Generated_Code/CS1.c \
../Generated_Code/Cpu.c \
../Generated_Code/DIS_PWM.c \
../Generated_Code/FAT1.c \
../Generated_Code/FRTOS1.c \
../Generated_Code/LEDG.c \
../Generated_Code/LEDR.c \
../Generated_Code/LEDpin1.c \
../Generated_Code/LEDpin2.c \
../Generated_Code/MINI1.c \
../Generated_Code/PE_LDD.c \
../Generated_Code/PwmLdd1.c \
../Generated_Code/PwmLdd2.c \
../Generated_Code/SD1.c \
../Generated_Code/SM1.c \
../Generated_Code/SMasterLdd1.c \
../Generated_Code/SS1.c \
../Generated_Code/TMOUT1.c \
../Generated_Code/TU1.c \
../Generated_Code/TU2.c \
../Generated_Code/TmDt1.c \
../Generated_Code/UTIL1.c \
../Generated_Code/Vectors.c \
../Generated_Code/WAIT1.c \
../Generated_Code/croutine.c \
../Generated_Code/event_groups.c \
../Generated_Code/ff.c \
../Generated_Code/heap_1.c \
../Generated_Code/heap_2.c \
../Generated_Code/heap_3.c \
../Generated_Code/heap_4.c \
../Generated_Code/heap_5.c \
../Generated_Code/list.c \
../Generated_Code/minGlue-FatFs.c \
../Generated_Code/minIni.c \
../Generated_Code/port.c \
../Generated_Code/queue.c \
../Generated_Code/tasks.c \
../Generated_Code/timers.c 

OBJS += \
./Generated_Code/AD1.o \
./Generated_Code/AS1.o \
./Generated_Code/ASerialLdd1.o \
./Generated_Code/AdcLdd1.o \
./Generated_Code/BitIoLdd1.o \
./Generated_Code/BitIoLdd2.o \
./Generated_Code/CHG_PWM.o \
./Generated_Code/CLS1.o \
./Generated_Code/CS1.o \
./Generated_Code/Cpu.o \
./Generated_Code/DIS_PWM.o \
./Generated_Code/FAT1.o \
./Generated_Code/FRTOS1.o \
./Generated_Code/LEDG.o \
./Generated_Code/LEDR.o \
./Generated_Code/LEDpin1.o \
./Generated_Code/LEDpin2.o \
./Generated_Code/MINI1.o \
./Generated_Code/PE_LDD.o \
./Generated_Code/PwmLdd1.o \
./Generated_Code/PwmLdd2.o \
./Generated_Code/SD1.o \
./Generated_Code/SM1.o \
./Generated_Code/SMasterLdd1.o \
./Generated_Code/SS1.o \
./Generated_Code/TMOUT1.o \
./Generated_Code/TU1.o \
./Generated_Code/TU2.o \
./Generated_Code/TmDt1.o \
./Generated_Code/UTIL1.o \
./Generated_Code/Vectors.o \
./Generated_Code/WAIT1.o \
./Generated_Code/croutine.o \
./Generated_Code/event_groups.o \
./Generated_Code/ff.o \
./Generated_Code/heap_1.o \
./Generated_Code/heap_2.o \
./Generated_Code/heap_3.o \
./Generated_Code/heap_4.o \
./Generated_Code/heap_5.o \
./Generated_Code/list.o \
./Generated_Code/minGlue-FatFs.o \
./Generated_Code/minIni.o \
./Generated_Code/port.o \
./Generated_Code/queue.o \
./Generated_Code/tasks.o \
./Generated_Code/timers.o 

C_DEPS += \
./Generated_Code/AD1.d \
./Generated_Code/AS1.d \
./Generated_Code/ASerialLdd1.d \
./Generated_Code/AdcLdd1.d \
./Generated_Code/BitIoLdd1.d \
./Generated_Code/BitIoLdd2.d \
./Generated_Code/CHG_PWM.d \
./Generated_Code/CLS1.d \
./Generated_Code/CS1.d \
./Generated_Code/Cpu.d \
./Generated_Code/DIS_PWM.d \
./Generated_Code/FAT1.d \
./Generated_Code/FRTOS1.d \
./Generated_Code/LEDG.d \
./Generated_Code/LEDR.d \
./Generated_Code/LEDpin1.d \
./Generated_Code/LEDpin2.d \
./Generated_Code/MINI1.d \
./Generated_Code/PE_LDD.d \
./Generated_Code/PwmLdd1.d \
./Generated_Code/PwmLdd2.d \
./Generated_Code/SD1.d \
./Generated_Code/SM1.d \
./Generated_Code/SMasterLdd1.d \
./Generated_Code/SS1.d \
./Generated_Code/TMOUT1.d \
./Generated_Code/TU1.d \
./Generated_Code/TU2.d \
./Generated_Code/TmDt1.d \
./Generated_Code/UTIL1.d \
./Generated_Code/Vectors.d \
./Generated_Code/WAIT1.d \
./Generated_Code/croutine.d \
./Generated_Code/event_groups.d \
./Generated_Code/ff.d \
./Generated_Code/heap_1.d \
./Generated_Code/heap_2.d \
./Generated_Code/heap_3.d \
./Generated_Code/heap_4.d \
./Generated_Code/heap_5.d \
./Generated_Code/list.d \
./Generated_Code/minGlue-FatFs.d \
./Generated_Code/minIni.d \
./Generated_Code/port.d \
./Generated_Code/queue.d \
./Generated_Code/tasks.d \
./Generated_Code/timers.d 


# Each subdirectory must supply rules for building sources it contributes
Generated_Code/%.o: ../Generated_Code/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -I"C:/Dark Angel/Documents/GitHub/BatteryTestSystem/BatteryTestSystemC/Static_Code/PDD" -I"C:/Dark Angel/Documents/GitHub/BatteryTestSystem/BatteryTestSystemC/Static_Code/IO_Map" -I"C:/Dark Angel/Documents/GitHub/BatteryTestSystem/BatteryTestSystemC/Sources" -I"C:/Dark Angel/Documents/GitHub/BatteryTestSystem/BatteryTestSystemC/Generated_Code" -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



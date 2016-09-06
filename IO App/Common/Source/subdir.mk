################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Common/Source/SMBus.c \
../Common/Source/adc.c \
../Common/Source/common.c \
../Common/Source/duplicate_checker.c \
../Common/Source/flash.c \
../Common/Source/input_string.c \
../Common/Source/modbus_ascii.c \
../Common/Source/sensor_driver.c 

OBJS += \
./Common/Source/SMBus.o \
./Common/Source/adc.o \
./Common/Source/common.o \
./Common/Source/duplicate_checker.o \
./Common/Source/flash.o \
./Common/Source/input_string.o \
./Common/Source/modbus_ascii.o \
./Common/Source/sensor_driver.o 

C_DEPS += \
./Common/Source/SMBus.d \
./Common/Source/adc.d \
./Common/Source/common.d \
./Common/Source/duplicate_checker.d \
./Common/Source/flash.d \
./Common/Source/input_string.d \
./Common/Source/modbus_ascii.d \
./Common/Source/sensor_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Common/Source/%.o: ../Common/Source/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cygwin C Compiler'
	gcc -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



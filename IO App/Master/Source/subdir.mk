################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Master/Source/Master.c 

OBJS += \
./Master/Source/Master.o 

C_DEPS += \
./Master/Source/Master.d 


# Each subdirectory must supply rules for building sources it contributes
Master/Source/%.o: ../Master/Source/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cygwin C Compiler'
	gcc -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


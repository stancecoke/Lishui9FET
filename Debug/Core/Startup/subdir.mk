################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32f031c6tx.s 

OBJS += \
./Core/Startup/startup_stm32f031c6tx.o 

S_DEPS += \
./Core/Startup/startup_stm32f031c6tx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/startup_stm32f031c6tx.o: ../Core/Startup/startup_stm32f031c6tx.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m0 -g3 -c -I"C:/Users/Martin/STM32CubeIDE/workspace_1.6.1/Lishui9FET/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -x assembler-with-cpp -MMD -MP -MF"Core/Startup/startup_stm32f031c6tx.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Components/Algorithm/user_lib.c 

C_DEPS += \
./Components/Algorithm/user_lib.d 

OBJS += \
./Components/Algorithm/user_lib.o 


# Each subdirectory must supply rules for building sources it contributes
Components/Algorithm/user_lib.o: ../Components/Algorithm/user_lib.c Components/Algorithm/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-D__FPU_USED=1U' -DSTM32F427xx -DARM_MATH_MATRIX_CHECK -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG -DARM_MATH_ROUNDING '-D__FPU_PRESENT=1U' -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Application -I../BSP -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Components/Controller -I../Components/Algorithm/Inc -I../Components/Algorithm -I../Components/Device -I../Components -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Components/Algorithm/user_lib.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"


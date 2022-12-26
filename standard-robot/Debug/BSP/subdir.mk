################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BSP/bsp_buzzer.c \
../BSP/bsp_can.c \
../BSP/bsp_fric.c \
../BSP/bsp_laser.c \
../BSP/bsp_led.c \
../BSP/bsp_rc.c \
../BSP/buffer.c \
../BSP/debugio.c \
../BSP/ringbuf.c 

CPP_SRCS += \
../BSP/crc_util.cpp 

C_DEPS += \
./BSP/bsp_buzzer.d \
./BSP/bsp_can.d \
./BSP/bsp_fric.d \
./BSP/bsp_laser.d \
./BSP/bsp_led.d \
./BSP/bsp_rc.d \
./BSP/buffer.d \
./BSP/debugio.d \
./BSP/ringbuf.d 

OBJS += \
./BSP/bsp_buzzer.o \
./BSP/bsp_can.o \
./BSP/bsp_fric.o \
./BSP/bsp_laser.o \
./BSP/bsp_led.o \
./BSP/bsp_rc.o \
./BSP/buffer.o \
./BSP/crc_util.o \
./BSP/debugio.o \
./BSP/ringbuf.o 

CPP_DEPS += \
./BSP/crc_util.d 


# Each subdirectory must supply rules for building sources it contributes
BSP/bsp_buzzer.o: ../BSP/bsp_buzzer.c BSP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-D__FPU_USED=1U' -DSTM32F427xx -DARM_MATH_MATRIX_CHECK -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG -DARM_MATH_ROUNDING '-D__FPU_PRESENT=1U' -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Application -I../BSP -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Components/Controller -I../Components/Algorithm/Inc -I../Components/Algorithm -I../Components/Device -I../Components -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"BSP/bsp_buzzer.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
BSP/bsp_can.o: ../BSP/bsp_can.c BSP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-D__FPU_USED=1U' -DSTM32F427xx -DARM_MATH_MATRIX_CHECK -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG -DARM_MATH_ROUNDING '-D__FPU_PRESENT=1U' -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Application -I../BSP -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Components/Controller -I../Components/Algorithm/Inc -I../Components/Algorithm -I../Components/Device -I../Components -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"BSP/bsp_can.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
BSP/bsp_fric.o: ../BSP/bsp_fric.c BSP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-D__FPU_USED=1U' -DSTM32F427xx -DARM_MATH_MATRIX_CHECK -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG -DARM_MATH_ROUNDING '-D__FPU_PRESENT=1U' -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Application -I../BSP -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Components/Controller -I../Components/Algorithm/Inc -I../Components/Algorithm -I../Components/Device -I../Components -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"BSP/bsp_fric.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
BSP/bsp_laser.o: ../BSP/bsp_laser.c BSP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-D__FPU_USED=1U' -DSTM32F427xx -DARM_MATH_MATRIX_CHECK -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG -DARM_MATH_ROUNDING '-D__FPU_PRESENT=1U' -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Application -I../BSP -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Components/Controller -I../Components/Algorithm/Inc -I../Components/Algorithm -I../Components/Device -I../Components -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"BSP/bsp_laser.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
BSP/bsp_led.o: ../BSP/bsp_led.c BSP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-D__FPU_USED=1U' -DSTM32F427xx -DARM_MATH_MATRIX_CHECK -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG -DARM_MATH_ROUNDING '-D__FPU_PRESENT=1U' -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Application -I../BSP -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Components/Controller -I../Components/Algorithm/Inc -I../Components/Algorithm -I../Components/Device -I../Components -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"BSP/bsp_led.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
BSP/bsp_rc.o: ../BSP/bsp_rc.c BSP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-D__FPU_USED=1U' -DSTM32F427xx -DARM_MATH_MATRIX_CHECK -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG -DARM_MATH_ROUNDING '-D__FPU_PRESENT=1U' -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Application -I../BSP -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Components/Controller -I../Components/Algorithm/Inc -I../Components/Algorithm -I../Components/Device -I../Components -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"BSP/bsp_rc.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
BSP/buffer.o: ../BSP/buffer.c BSP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-D__FPU_USED=1U' -DSTM32F427xx -DARM_MATH_MATRIX_CHECK -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG -DARM_MATH_ROUNDING '-D__FPU_PRESENT=1U' -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Application -I../BSP -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Components/Controller -I../Components/Algorithm/Inc -I../Components/Algorithm -I../Components/Device -I../Components -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"BSP/buffer.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
BSP/crc_util.o: ../BSP/crc_util.cpp BSP/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 '-D__FPU_USED=1U' -DSTM32F427xx -DARM_MATH_MATRIX_CHECK -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG -DARM_MATH_ROUNDING '-D__FPU_PRESENT=1U' -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Application -I../BSP -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Components -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"BSP/crc_util.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
BSP/debugio.o: ../BSP/debugio.c BSP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-D__FPU_USED=1U' -DSTM32F427xx -DARM_MATH_MATRIX_CHECK -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG -DARM_MATH_ROUNDING '-D__FPU_PRESENT=1U' -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Application -I../BSP -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Components/Controller -I../Components/Algorithm/Inc -I../Components/Algorithm -I../Components/Device -I../Components -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"BSP/debugio.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
BSP/ringbuf.o: ../BSP/ringbuf.c BSP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-D__FPU_USED=1U' -DSTM32F427xx -DARM_MATH_MATRIX_CHECK -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG -DARM_MATH_ROUNDING '-D__FPU_PRESENT=1U' -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Application -I../BSP -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Components/Controller -I../Components/Algorithm/Inc -I../Components/Algorithm -I../Components/Device -I../Components -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"BSP/ringbuf.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"


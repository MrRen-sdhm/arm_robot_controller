cmake_minimum_required(VERSION 3.10)
project(arm_robot_f427_v3.0 C CXX ASM)
set(CMAKE_CXX_STANDARD 17)

add_definitions(-DARM_MATH_CM4 -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1)
link_directories(Drivers/CMSIS/Lib/GCC)

include_directories(
        User
        User/BMX055
        User/BSP
        User/BSP/arm
        User/BSP/base
        User/BSP/hand
        User/BSP/head
        User/BSP/lift
        User/rosserial
        User/RML/include
        User/eigen3
        Inc
        Drivers/STM32F4xx_HAL_Driver/Inc
        Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
        Drivers/CMSIS/Include
        Drivers/CMSIS/Device/ST/STM32F4xx/Include
        Middlewares/Third_Party/LwIP/src/include
        Middlewares/Third_Party/LwIP/system
        Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
        Middlewares/Third_Party/FreeRTOS/Source/include
        Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
)

file(REMOVE Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c)

file(GLOB_RECURSE SOURCES
        "Middlewares/*.*"
        "Drivers/STM32F4xx_HAL_Driver/*.*"
        "Src/*.*"
        "User/*.*"
)

# if enabled LTO, startup asm must be place firstly to avoid linker removing ISR code
add_executable(${PROJECT_NAME}.elf startup/startup_stm32f427xx.s ${SOURCES} ${LINKER_SCRIPT})
target_link_libraries(${PROJECT_NAME}.elf libarm_cortexM4lf_math.a)

#include(CheckIPOSupported)
#check_ipo_supported(LANGUAGES C)
#check_ipo_supported(LANGUAGES CXX)
#set_property(INTERPROCEDURAL_OPTIMIZATION TRUE)

# 在编译完成后, 生成 hex 与 bin 文件
#set(HEX_FILE ${PROJECT_NAME}.hex)
#set(BIN_FILE ${PROJECT_NAME}.bin)
#add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
#        # COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
#        COMMAND I:/SysGCC/gcc-arm-none-eabi-7-2018-q2-update-win32/bin/arm-none-eabi-objcopy.exe -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
#        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
#        COMMENT "Building ${HEX_FILE}
#Building ${BIN_FILE}")


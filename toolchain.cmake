
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR cortex-m4)

#set(TOOLCHAIN_PATH "I:/SysGCC/gcc-arm-none-eabi-7-2018-q2-update-win32")
set(TOOLCHAIN_PATH "/opt/gcc-arm-none-eabi-8-2018-q4-major")

set(CMAKE_C_COMPILER "${TOOLCHAIN_PATH}/bin/arm-none-eabi-gcc")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_PATH}/bin/arm-none-eabi-g++")

set(cpu_flags "-mcpu=cortex-m4 -mthumb -mthumb-interwork -mfloat-abi=hard -mfpu=fpv4-sp-d16")

# lto is not compele, do not enable lto
set(c_flags "-ffunction-sections -fdata-sections -g -fno-common -fmessage-length=0 -specs=nosys.specs -fno-builtin")
set(cxx_flags "-ffunction-sections -fdata-sections -g -fno-common -fmessage-length=0 -specs=nosys.specs -fno-builtin")

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F427VITx_FLASH.ld)

set(CMAKE_C_FLAGS "${c_flags} ${cpu_flags}")
set(CMAKE_CXX_FLAGS "${cxx_flags} ${cpu_flags}")
set(CMAKE_ASM_FLAGS "${cpu_flags}")
set(CMAKE_EXE_LINKER_FLAGS "${cpu_flags} -Wl,--gc-sections,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map -T ${LINKER_SCRIPT}")

add_definitions(-D__weak=__attribute__\(\(weak\)\) -D__packed=__attribute__\(\(__packed__\)\) -DUSE_HAL_DRIVER -DSTM32F427xx)

set(CMAKE_FIND_ROOT_PATH "${TOOLCHAIN_PATH}")
# search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
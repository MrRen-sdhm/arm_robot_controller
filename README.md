## 开发环境配置

开发环境：Ubuntu + CLion

GNU Arm Embedded Toolchain下载：[https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads#](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads#)
安装：下载完成后解压到opt文件夹

##### 修改工程中的两个文件：

1：toolchains.cmake:
修改TOOLCHAIN_PATH：set(TOOLCHAIN_PATH "/opt/gcc-arm-none-eabi-8-2018-q4-major")

2：CmakeLists.txt：
注释以下行：

```
# 在编译完成后, 生成 hex 与 bin 文件
set(HEX_FILE ${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_NAME}.bin)
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        # COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND I:/SysGCC/gcc-arm-none-eabi-7-2018-q2-update-win32/bin/arm-none-eabi-objcopy.exe -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")
```

##### 修改构建选项：

File->settinges->Build, Execution, Deployment
Build Type：Debug
CMake options:
-DCMAKE_TOOLCHAIN_FILE=toolchain.cmake

##### 添加External Tools：

File->settinges->Tools->External Tools->添加：

###### 1：SEGGER_JLINK_GDB:（用于调试）

1.选择Program：
/opt/gcc-arm-none-eabi-8-2018-q4-major/bin/arm-none-eabi-gdb
2.修改argument：

```
$CmakeCurrentTargetName$
-iex
"target remote tcp:192.168.123.2:2331"
```

3.选择Working Directory（点击Insert Macro...）:$CMakeCurrentBuildDir$

###### 2：Flash：（用于下载程序）

1. 选择Program：
   /opt/gcc-arm-none-eabi-8-2018-q4-major/bin/arm-none-eabi-gdb
2. 修改argument：

```
$CMakeCurrentTargetName$
-iex
"target remote tcp:192.168.123.2:2331"
-iex
"file $CMakeCurrentTargetName$"
-iex
"monitor reset"
-iex
"monitor halt"
-iex
"monitor waithalt 1000"
-iex
"load"
-iex
"monitor go"
-iex
"quit"
```

​    3. 选择Working Directory（点击Insert Macro...）:$CMakeCurrentBuildDir$

## 构建说明

1. 使用 STM32CubeMX 打开 arm_robot_xxxx.ioc, 生成项目
2. 在当前目录下, 执行`python patch-src`, 打代码补丁以兼容 gcc 编译器
3. 在 Clion 中 Import Project

## 注意事项

1. 增加 .c 或 .cpp 等源文件后, 需要调用 Clion 中的 Tools -> CMake -> Reload CMake Project
2. 修改 arm_robot_xxxx.ioc 项目后, 需要重新执行步骤 1、2

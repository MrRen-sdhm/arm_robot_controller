# 构建说明

1. 使用 STM32CubeMX 打开 arm_robot_xxxx.ioc, 生成项目
2. 在当前目录下, 执行`python patch-src`, 打代码补丁以兼容 gcc 编译器
3. 在 Clion 中 Import Project

# 注意事项

1. 增加 .c 或 .cpp 等源文件后, 需要调用 Clion 中的 Tools -> CMake -> Reload CMake Project
2. 修改 arm_robot_xxxx.ioc 项目后, 需要重新执行步骤 1、2

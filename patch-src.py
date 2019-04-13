#coding=utf8
'''修改STM32CubeMX生成的源代码, 以满足项目需求
'''

def file_replace(path, old, new):
    text = open(path, 'r', encoding='utf8').read()
    text = text.replace(old, new)
    open(path, 'w', encoding='utf8').write(text)
    print("Patch", path)

if __name__ == '__main__':
    file_replace(
        'Middlewares/Third_Party/LwIP/system/arch/cc.h', 
        '\n#define LWIP_PROVIDE_ERRNO', '''\
// #define LWIP_PROVIDE_ERRNO
#include <sys/errno.h>'''
    )

    file_replace(
        'Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f427xx.h',
        '#define __FPU_PRESENT             1U', '''\
#ifndef __FPU_PRESENT
#define __FPU_PRESENT 1U
#endif'''
    )

    file_replace(
        'Drivers/CMSIS/Include/cmsis_gcc.h',
        'register uint32_t result;', 'uint32_t result;'
    )

    input("Press any key to exit")


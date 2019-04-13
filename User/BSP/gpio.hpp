#pragma once

#include <list>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"

namespace hustac {

class GPIOOutput {
private:
    static std::list<GPIOOutput*> instances;
public:
    static void init_pin(const char* name=NULL);        // 初始化引脚为默认输出
    static int write_pin(const char* name, bool state); // 写指定引脚
    static int toggle_pin(const char* name);            // 切换指定引脚状态
    static GPIOOutput* get_pin(const char* name);       // 获取指定引脚

private:
    const char* name;
    GPIO_TypeDef* gpio_port;
    const uint32_t pin;
    const GPIO_PinState active_state;
public:
    GPIOOutput(const char* _name, GPIO_TypeDef* _gpio_port, uint32_t _pin, GPIO_PinState _active_state) :
        name(_name), gpio_port(_gpio_port), pin(_pin), active_state(_active_state) 
    {
        instances.push_back(this);
    }
    
    void on() {
        set(true);
    }
    
    void off() {
        set(false);
    }
    
    void set(bool state) {
        if (state ^ (bool)active_state) {
            HAL_GPIO_WritePin(gpio_port, pin, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(gpio_port, pin, GPIO_PIN_SET);
        }
    }
    
    void toggle() {
        HAL_GPIO_TogglePin(gpio_port, pin);
    }
    
    bool get() {
        return bool(gpio_port->ODR & pin) ^ (bool)active_state;
    }
    
};

#define WritePin(name, state)   GPIOOutput::write_pin(#name, (state))
#define TogglePin(name)         GPIOOutput::toggle_pin(#name)
#define GetPin(name)            GPIOOutput::get_pin(#name)

//extern GPIOOutput LED1;
//extern GPIOOutput LED2;
//extern GPIOOutput LED_STATUS1;
//extern GPIOOutput LED_STATUS2;
//extern GPIOOutput LED_START;
//extern GPIOOutput LED_STOP;
//extern GPIOOutput LED_RESET;
//extern GPIOOutput BEEP;
//extern GPIOOutput LIFT_UNLOCK;
//extern GPIOOutput LIFT_DISABLE;
//extern GPIOOutput LIFT_DIR_DOWN;
//extern GPIOOutput LIFT_PULSE;
//extern GPIOOutput RS485_1_WE;
//extern GPIOOutput RS485_2_WE;
//extern GPIOOutput RS485_3_WE;

}

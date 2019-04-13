#include <cstring>

#include "gpio.hpp"

namespace hustac {

std::list<GPIOOutput*> GPIOOutput::instances;

void GPIOOutput::init_pin(const char* name) {
    for (GPIOOutput* instance : instances) {
        if (!name || strcmp(instance->name, name) == 0) {
            instance->off();
        }
    }
}

int GPIOOutput::write_pin(const char* name, bool state) {
    for (GPIOOutput* instance : instances) {
        if (!name || strcmp(instance->name, name) == 0) {
            instance->set(state);
            return 0;
        }
    }
    return -1;
}
int GPIOOutput::toggle_pin(const char* name) {
    for (GPIOOutput* instance : instances) {
        if (!name || strcmp(instance->name, name) == 0) {
            instance->toggle();
            return 0;
        }
    }
    return -1;
};

GPIOOutput* GPIOOutput::get_pin(const char* name) {
    for (GPIOOutput* instance : instances) {
        if (strcmp(instance->name, name) == 0) {
            return instance;
        }
    }
    return NULL;
}

//#define define_gpio_output(name, active_state) \
//    GPIOOutput name(#name, name##_GPIO_Port, name##_Pin, active_state);

//define_gpio_output(LED1, GPIO_PIN_RESET);
//define_gpio_output(LED2, GPIO_PIN_RESET);
//define_gpio_output(LED_STATUS1, GPIO_PIN_SET);
//define_gpio_output(LED_STATUS2, GPIO_PIN_SET);
//define_gpio_output(LED_START, GPIO_PIN_SET);
//define_gpio_output(LED_STOP, GPIO_PIN_SET);
//define_gpio_output(LED_RESET, GPIO_PIN_SET);
//define_gpio_output(BEEP, GPIO_PIN_SET);
//define_gpio_output(LIFT_UNLOCK, GPIO_PIN_RESET); // on为解锁, off为锁定
//define_gpio_output(LIFT_DISABLE, GPIO_PIN_SET); // on为禁用电机, off为启用电机
//define_gpio_output(LIFT_DIR_DOWN, GPIO_PIN_SET); // on为下降, off为上升
//define_gpio_output(LIFT_PULSE, GPIO_PIN_SET); // on为有效电平(高电平), off为无效电平(低电平)
//define_gpio_output(RS485_1_WE, GPIO_PIN_SET);
//define_gpio_output(RS485_2_WE, GPIO_PIN_SET);
//define_gpio_output(RS485_3_WE, GPIO_PIN_SET);

//#undef define_gpio_output
}

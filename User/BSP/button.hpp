/*
 * button.hpp
 *
 *  Created on: 2018年6月3日
 *      Author: shuixiang
 */

#pragma once

#include <list>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"

namespace hustac {

class Button {
private:
    static std::list<Button*> instances;    // 保存所有按键实例
public:
    static void init_button(const char* name=NULL);   // 更新按键状态
    static void update_button(const char* name=NULL);   // 更新按键状态
    static Button* get_button(const char* name);
    
public:
    const char* name;
    GPIO_TypeDef* gpiox;
    const uint16_t gpio_pin;
    const GPIO_PinState active_state;
    const int delay_keystroke;      // ms
    const int delay_hold;           // 触发长按事件的延迟 , =0则不触发长按 ms
    const int delay_hold_repeat;    // 长按后重复触发短按的间隔, =0则不重复短按 ms

    enum class PressState {
        RELEASED,
        PENDING_PRESSED,
        PRESSED,
        PENDING_RELEASED,
    };
    PressState state_press;       // 当前按键状态
    uint32_t last_state_press_change;

    enum class HoldState {
        NO_PRESSED,
        PRESSED,
        PENDING_HOLD,
        HOLDING,
        PENDING_REPEAT,
        REPEATING,
    };
    HoldState state_hold;        // 当前长按状态
    uint32_t last_state_hold_change;

    int count_key_down;     // 按键被按下次数
    int count_key_press;    // 按键被短按次数
    int count_key_hold;     // 按键被长按次数
    int count_key_up;       // 按键被松开次数

    Button(const char* _name, GPIO_TypeDef *_gpiox, uint16_t _gpio_pin,
            GPIO_PinState _active_state = GPIO_PIN_RESET, int _delay_keystroke =
            20, int _delay_hold = 0, int _delay_hold_repeat = 0) :
                name(_name), gpiox(_gpiox), gpio_pin(_gpio_pin), active_state(_active_state), delay_keystroke(
                _delay_keystroke), delay_hold(_delay_hold), delay_hold_repeat(
                _delay_hold_repeat) {
        instances.push_back(this);
        reset();
    }
                
    void init() {
        reset();
        if (is_pin_actived()) {
            state_press = PressState::PRESSED;
            last_state_press_change = HAL_GetTick();
            count_key_down++;
            count_key_press++;
        }
    }

    void reset() {
        state_press = PressState::RELEASED;
        last_state_press_change = HAL_GetTick();
        state_hold = HoldState::NO_PRESSED;
        last_state_hold_change = HAL_GetTick();
        count_key_down = 0;
        count_key_press = 0;
        count_key_hold = 0;
        count_key_up = 0;
    }

    bool is_pin_actived() {
        return HAL_GPIO_ReadPin(gpiox, gpio_pin) == active_state;
    }

#undef _FSM_DEBUG
    
    void _FSM_press() {
        
#define CHANGE_MODE(new_mode)   do {\
    state_press = PressState::new_mode;\
    last_state_press_change = HAL_GetTick();\
    /*printf("[BUTTOM FSM PRESS]->" #new_mode "\n");*/\
} while (0);
        
        bool stop_loop = false;
        while (!stop_loop) {
            switch (state_press) {
            case PressState::RELEASED:
                if (is_pin_actived()) {
                    CHANGE_MODE(PENDING_PRESSED);
                } else {
                    stop_loop = true;
                }
                break;
            case PressState::PENDING_PRESSED:
                if (is_pin_actived()) {
                    if (HAL_GetTick() - last_state_press_change >= (uint32_t)delay_keystroke) {
                        CHANGE_MODE(PRESSED);
                        count_key_down++;
                    } else {
                        stop_loop = true;
                    }
                } else {
                    CHANGE_MODE(RELEASED);
                }
                break;
            case PressState::PRESSED:
                if (!is_pin_actived()) {
                    CHANGE_MODE(PENDING_RELEASED);
                } else {
                    stop_loop = true;
                }
                break;
            case PressState::PENDING_RELEASED:
                if (!is_pin_actived()) {
                    if (HAL_GetTick() - last_state_press_change >= (uint32_t)delay_keystroke) {
                        CHANGE_MODE(RELEASED);
                        count_key_up++;
                    } else {
                        stop_loop = true;
                    }
                } else {
                    CHANGE_MODE(PRESSED);
                }
                break;
            }
        }
        
#undef CHANGE_MODE
        
    }

    bool is_pressed() {
        return state_press == PressState::PRESSED || state_press == PressState::PENDING_RELEASED;
    }

    void _FSM_simple() {
        
#define CHANGE_MODE(new_mode)   do {\
    state_hold = HoldState::new_mode;\
    last_state_hold_change = HAL_GetTick();\
    /*printf("[BUTTOM FSM SIMPLE]->" #new_mode "\n");*/\
} while (0);

        bool stop_loop = false;
        while (!stop_loop) {
            switch (state_hold) {
            case HoldState::NO_PRESSED:
                if (is_pressed()) {
                    CHANGE_MODE(PRESSED);
                    count_key_press++;
                } else {
                    stop_loop = true;
                }
                break;
            case HoldState::PRESSED:
                if (!is_pressed()) {
                    CHANGE_MODE(NO_PRESSED);
                } else {
                    stop_loop = true;
                }
                break;
            default:
                stop_loop = true;
            }
        }
        
#undef CHANGE_MODE
    }

    void _FSM_hold() {
        
#define CHANGE_MODE(new_mode)   do {\
    state_hold = HoldState::new_mode;\
    last_state_hold_change = HAL_GetTick();\
    /*printf("[BUTTOM FSM HOLD]->" #new_mode "\n");*/\
} while (0);

        bool stop_loop = false;
        while (!stop_loop) {
            switch (state_hold) {
            case HoldState::NO_PRESSED:
                if (is_pressed()) {
                    CHANGE_MODE(PENDING_HOLD);
                } else {
                    stop_loop = true;
                }
                break;
            case HoldState::PENDING_HOLD:
                if (is_pressed()) {
                    if (HAL_GetTick() - last_state_hold_change >= (uint32_t)delay_hold) {
                        CHANGE_MODE(HOLDING);
                        count_key_hold++;
                    } else {
                        stop_loop = true;
                    }
                } else {
                    CHANGE_MODE(NO_PRESSED);
                    count_key_press++;
                }
                break;
            case HoldState::HOLDING:
                if (!is_pressed()) {
                    CHANGE_MODE(NO_PRESSED);
                } else {
                    stop_loop = true;
                }
                break;
            default:
                stop_loop = true;
            }
        }
        
#undef CHANGE_MODE
    }

    void _FSM_repeat() {
        
#define CHANGE_MODE(new_mode)   do {\
    state_hold = HoldState::new_mode;\
    last_state_hold_change = HAL_GetTick();\
    /*printf("[BUTTOM FSM REPEAT]->" #new_mode "\n");*/\
} while (0);

        bool stop_loop = false;
        while (!stop_loop) {
            switch (state_hold) {
            case HoldState::NO_PRESSED:
                if (is_pressed()) {
                    CHANGE_MODE(PENDING_REPEAT);
                    count_key_press++;
                } else {
                    stop_loop = true;
                }
                break;
            case HoldState::PENDING_REPEAT:
                if (is_pressed()) {
                    if (HAL_GetTick() - last_state_hold_change >= (uint32_t)delay_hold) {
                        CHANGE_MODE(REPEATING);
                        if (delay_hold >= delay_hold_repeat) {
                            count_key_press++;
                        }
                    } else {
                        stop_loop = true;
                    }
                } else {
                    CHANGE_MODE(NO_PRESSED);
                }
                break;
            case HoldState::REPEATING:
                if (!is_pressed()) {
                    CHANGE_MODE(NO_PRESSED);
                } else {
                    if (HAL_GetTick() - last_state_hold_change >= (uint32_t)delay_hold_repeat) {
                        last_state_hold_change += delay_hold_repeat;
                        count_key_press++;
                    } else {
                        stop_loop = true;
                    }
                }
                break;
            default:
                stop_loop = true;
            }
        }
        
#undef CHANGE_MODE
    }

    void update() {
        _FSM_press();
        if (delay_hold == 0 && delay_hold_repeat == 0) {
            _FSM_simple();
        } else if (delay_hold > 0 && delay_hold_repeat == 0) {
            _FSM_hold();
        } else if (delay_hold_repeat > 0) {
            _FSM_repeat();
        }
    }

};

}

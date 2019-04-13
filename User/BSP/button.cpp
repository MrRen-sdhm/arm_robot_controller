/*
 * button.cpp
 *
 *  Created on: 2018年6月3日
 *      Author: shuixiang
 */

#include <cstring>
#include "main.h"

#include "button.hpp"

namespace hustac {
    
std::list<Button*> Button::instances;

void Button::init_button(const char* name) {
    for (Button* button : instances) {
        if (!name || strcmp(button->name, name)) {
            button->init();
        }
    }
}

void Button::update_button(const char* name) {
    for (Button* button : instances) {
        if (!name || strcmp(button->name, name)) {
            button->update();
        }
    }
}

Button* Button::get_button(const char* name) {
    for (Button* button : instances) {
        if (strcmp(button->name, name)) {
            return button;
        }
    }
    return NULL;
}

}


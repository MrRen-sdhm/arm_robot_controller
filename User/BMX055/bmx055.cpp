/*
 * bmx055.cpp
 *
 *  Created on: 2018年5月30日
 *      Author: shuixiang
 */

#include "i2c.h"

#include "bmx055.hpp"

namespace hustac {

BMX055 bmx055_camera(&hi2c1, false, false, false);

}


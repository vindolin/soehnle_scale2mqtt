#pragma once
#include <Arduino.h>
#include "measurement_utils.h"

std::map<int, User> users = {
    // name, name, age, height, is_male, activity_level (1-5)
    {1, {"mona", 50, 159, false, 2}},
    {2, {"tulpe", 55, 180, true, 2}}
};

uint8_t userCount = static_cast<uint8_t>(users.size());

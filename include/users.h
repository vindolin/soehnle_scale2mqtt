#pragma once
#include <Arduino.h>
#include "measurement_utils.h"

std::map<String, User> users = {
    // name, age, height, is_male, activity_level (1-5)
    {"mona", {50, 159, false, 2}},
    {"tulpe", {55, 180, true, 2}}
};

std::vector<UserDetectionRule> detectionRules = {
    // name, minWeightKg, maxWeightKg
    {"mona", 45, 74},
    {"tulpe", 75, 89}
};

uint8_t userCount = static_cast<uint8_t>(detectionRules.size());

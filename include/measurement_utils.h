#pragma once
#include <map>
#include <vector>

struct User {
    String name;
    int age;
    float height;
    bool isMale;
    int activityLevel;
};

struct Measurement {
    String user;
    String time;
    uint8_t pID = 0;
    float weight = 0.0;
    float fat = 0.0;
    float water = 0.0;
    float muscle = 0.0;
};

struct MeasurementFrame {
    uint8_t pID;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    float weightKg;
    uint16_t imp5;
    uint16_t imp50;
};

struct UserDetectionRule {
    String name;
    float minWeightKg;
    float maxWeightKg;
};

// Global configuration for users and rules
extern std::map<String, User> Users;
extern std::vector<UserDetectionRule> detectionRules;

constexpr auto MEASUREMENT_OPCODE = 0x09;

constexpr auto MEASUREMENT_FRAME_LENGTH = 15;

// Calculations based on OpenScale implementation
// https://github.com/oliexdev/openScale/blob/master/android_app/app/src/main/java/com/health/openscale/core/bluetooth/scales/SoehnleHandler.kt
float calculateFat(const User& user, float weight, float imp50) {
    float activityCorrFac = 0.0;
    if (user.activityLevel == 4) activityCorrFac = user.isMale ? 2.5 : 2.3;
    else if (user.activityLevel == 5) activityCorrFac = user.isMale ? 4.3 : 4.1;

    float sexCorrFac = user.isMale ? 0.250 : 0.214;
    float activitySexDiv = user.isMale ? 65.5 : 55.1;

    return (1.847 * weight * 10000.0 / (user.height * user.height) + 
            sexCorrFac * user.age + 0.062 * imp50 - 
            (activitySexDiv - activityCorrFac));
}

float calculateWater(const User& user, float weight, float imp50) {
    float activityCorrFac = 0.0;
    if (user.activityLevel >= 1 && user.activityLevel <= 3) activityCorrFac = user.isMale ? 2.83 : 0.0;
    else if (user.activityLevel == 4) activityCorrFac = user.isMale ? 3.93 : 0.4;
    else if (user.activityLevel == 5) activityCorrFac = user.isMale ? 5.33 : 1.4;

    return ((0.3674 * user.height * user.height / imp50 + 
            0.17530 * weight - 0.11 * user.age + 
            (6.53 + activityCorrFac)) / weight * 100.0);
}

float calculateMuscle(const User& user, float weight, float imp50, float imp5) {
    float activityCorrFac = 0.0;
    if (user.activityLevel >= 1 && user.activityLevel <= 3) activityCorrFac = user.isMale ? 3.6224 : 0.0;
    else if (user.activityLevel == 4) activityCorrFac = user.isMale ? 4.3904 : 0.0;
    else if (user.activityLevel == 5) activityCorrFac = user.isMale ? 5.4144 : 1.664;

    return (((0.47027 / imp50 - 0.24196 / imp5) * user.height * user.height + 
            0.13796 * weight - 0.1152 * user.age + 
            (5.12 + activityCorrFac)) / weight * 100.0);
}

bool parseMeasurementFrame(const uint8_t* data, size_t length, MeasurementFrame& frame) {
    if (length != MEASUREMENT_FRAME_LENGTH || data[0] != MEASUREMENT_OPCODE) {
        return false;
    }

    frame.pID = data[1];
    frame.year = (data[2] << 8) | data[3];
    frame.month = data[4];
    frame.day = data[5];
    frame.hour = data[6];
    frame.minute = data[7];
    frame.second = data[8];
    frame.weightKg = static_cast<float>((data[9] << 8) | data[10]) / 10.0f;
    frame.imp5 = (data[11] << 8) | data[12];
    frame.imp50 = (data[13] << 8) | data[14];
    return true;
}

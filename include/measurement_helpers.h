#pragma once

#include <Arduino.h>
#include <map>
#include <vector>
#include <time.h>

struct User {
    int age;
    float height;
    bool isMale;
    int activityLevel;
};

std::map<int, User> users = {
    // age, height, is_male, activity_level (1-5)
    {1, {50, 159, false, 2}},
    {2, {55, 180, true, 2}}
};

uint8_t userCount = static_cast<uint8_t>(users.size());

struct Measurement {
    String time;
    uint8_t pID = 0;
    float weight = 0.0;
    float fat = 0.0;
    float water = 0.0;
    float muscle = 0.0;
};

Measurement latestMeasurement;

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


constexpr uint16_t BODY_COMP_FLAG_TIMESTAMP_PRESENT = 0x0002;
constexpr uint16_t BODY_COMP_FLAG_USER_ID_PRESENT = 0x0004;
constexpr uint16_t BODY_COMP_FLAG_MUSCLE_PERCENT = 0x0010;
constexpr uint16_t BODY_COMP_FLAG_BODY_WATER_MASS = 0x0100;
constexpr uint16_t BODY_COMP_FLAG_IMPEDANCE = 0x0200;
constexpr uint16_t BODY_COMP_FLAG_WEIGHT = 0x0400;

float decodeMassKg(uint16_t raw) {
    if (raw == 0) {
        return 0.0f;
    }

    float kg = raw * 0.005f;
    if (kg < 10.0f) {
        // Soehnle appears to use 0.1 kg resolution despite the spec stating
        // 0.005 kg.
        kg = raw * 0.1f;
    }
    return kg;
}

bool buildMeasurementFromBodyCompositionFrame(const uint8_t *data, size_t length, Measurement &outMeasurement) {
    if (length < 4) {
        return false;
    }

    size_t offset = 0;
    auto readUInt16 = [&](uint16_t &value) -> bool {
        if (offset + 2 > length) {
            Serial.println("Body composition payload truncated (uint16)");
            return false;
        }
        value = static_cast<uint16_t>(data[offset]) | (static_cast<uint16_t>(data[offset + 1]) << 8);
        offset += 2;
        return true;
    };

    const uint16_t flags = static_cast<uint16_t>(data[offset]) | (static_cast<uint16_t>(data[offset + 1]) << 8);
    offset += 2;

    uint16_t bodyFatRaw = 0;
    if (!readUInt16(bodyFatRaw)) {
        return false;
    }
    float bodyFatPercent = bodyFatRaw / 10.0f;

    uint16_t year = 0;
    uint8_t month = 1;
    uint8_t day = 1;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    if (flags & BODY_COMP_FLAG_TIMESTAMP_PRESENT) {
        if (offset + 7 > length) {
            Serial.println("Body composition payload truncated (timestamp)");
            return false;
        }
        year = static_cast<uint16_t>(data[offset]) | (static_cast<uint16_t>(data[offset + 1]) << 8);
        month = data[offset + 2];
        day = data[offset + 3];
        hour = data[offset + 4];
        minute = data[offset + 5];
        second = data[offset + 6];
        offset += 7;
    } else {
        time_t now = time(nullptr);
        struct tm timeinfo;
        localtime_r(&now, &timeinfo);
        year = static_cast<uint16_t>(timeinfo.tm_year + 1900);
        month = static_cast<uint8_t>(timeinfo.tm_mon + 1);
        day = static_cast<uint8_t>(timeinfo.tm_mday);
        hour = static_cast<uint8_t>(timeinfo.tm_hour);
        minute = static_cast<uint8_t>(timeinfo.tm_min);
        second = static_cast<uint8_t>(timeinfo.tm_sec);
    }

    uint8_t userId = 0xFF;
    if (flags & BODY_COMP_FLAG_USER_ID_PRESENT) {
        if (offset >= length) {
            Serial.println("Body composition payload truncated (user id)");
            return false;
        }
        userId = data[offset++];
    }

    if (userId == 0xFF) {
        Serial.println("Body composition payload missing user id");
        return false;
    }

    auto userIt = users.find(userId);
    if (userIt == users.end()) {
        Serial.printf("Unknown user pID: %d\n", userId);
        return false;
    }

    float musclePercent = 0.0f;
    bool hasMusclePercent = false;
    if (flags & BODY_COMP_FLAG_MUSCLE_PERCENT) {
        uint16_t raw = 0;
        if (!readUInt16(raw)) {
            return false;
        }
        musclePercent = raw / 10.0f;
        hasMusclePercent = true;
    }

    float bodyWaterMassKg = 0.0f;
    bool hasBodyWaterMass = false;
    if (flags & BODY_COMP_FLAG_BODY_WATER_MASS) {
        uint16_t raw = 0;
        if (!readUInt16(raw)) {
            return false;
        }
        bodyWaterMassKg = decodeMassKg(raw);
        hasBodyWaterMass = true;
    }

    float impedanceOhms = 0.0f;
    if (flags & BODY_COMP_FLAG_IMPEDANCE) {
        uint16_t raw = 0;
        if (!readUInt16(raw)) {
            return false;
        }
        impedanceOhms = raw / 10.0f;
    }

    float weightKg = 0.0f;
    bool hasWeight = false;
    if (flags & BODY_COMP_FLAG_WEIGHT) {
        uint16_t raw = 0;
        if (!readUInt16(raw)) {
            return false;
        }
        weightKg = decodeMassKg(raw);
        hasWeight = true;
    }

    (void)impedanceOhms; // Currently unused for derived metrics.

    char timeStr[25];
    snprintf(timeStr, sizeof(timeStr), "%04u-%02u-%02uT%02u:%02u:%02uZ",
             year, month, day, hour, minute, second);

    outMeasurement = Measurement();
    outMeasurement.pID = userId;
    outMeasurement.time = timeStr;
    outMeasurement.weight = hasWeight ? weightKg : 0.0f;
    outMeasurement.fat = bodyFatPercent;
    outMeasurement.muscle = hasMusclePercent ? musclePercent : 0.0f;
    if (hasBodyWaterMass && hasWeight && weightKg > 0.0f) {
        outMeasurement.water = (bodyWaterMassKg / weightKg) * 100.0f;
    } else {
        outMeasurement.water = 0.0f;
    }

    return true;
}

void logAndStoreMeasurement(const Measurement &measurement) {
    Serial.printf("personID %d - %s: weight:%4.1fkg, fat:%4.1f%%, water:%4.1f%%, muscle:%4.1f%%\n",
                  measurement.pID,
                  measurement.time.c_str(),
                  measurement.weight,
                  measurement.fat,
                  measurement.water,
                  measurement.muscle);

    if (latestMeasurement.time.isEmpty() || measurement.time > latestMeasurement.time) {
        latestMeasurement = measurement;
    }
}

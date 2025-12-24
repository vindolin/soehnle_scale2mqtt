#pragma once

#include <Arduino.h>
#include <time.h>

#include "measurement_utils.h"
#include "users.h"


extern Measurement latestMeasurement;

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

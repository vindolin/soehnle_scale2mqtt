#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

struct Measurement {
    char time[25] = "";
    uint8_t pID = 0;
    float weightKg = 0.0;
    float fatPercentage = 0.0;
    float waterPercentage = 0.0;
    float musclePercentage = 0.0;
};

Measurement measurement;

constexpr auto MEASUREMENT_OPCODE = 0x09;
constexpr auto MEASUREMENT_FRAME_LENGTH = 15;

constexpr uint16_t BODY_COMP_FLAG_TIMESTAMP_PRESENT = 0x0002;
constexpr uint16_t BODY_COMP_FLAG_USER_ID_PRESENT = 0x0004;
constexpr uint16_t BODY_COMP_FLAG_BASAL_METABOLISM_PRESENT = 0x0008;
constexpr uint16_t BODY_COMP_FLAG_MUSCLE_PERCENT = 0x0010;
constexpr uint16_t BODY_COMP_FLAG_BODY_WATER_MASS = 0x0100;
constexpr uint16_t BODY_COMP_FLAG_WEIGHT = 0x0400;

float decodeMassKg(uint16_t raw) {
    if (raw == 0) {
        return 0.0f;
    }

    float kg = raw * 0.1f;
    return kg;
}

bool buildMeasurementFromBodyCompositionFrame(const uint8_t *data, size_t length) {
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

    if (flags & BODY_COMP_FLAG_BASAL_METABOLISM_PRESENT) {
        uint16_t raw = 0;
        if (!readUInt16(raw)) {
            return false;
        }
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

    snprintf(measurement.time, sizeof(measurement.time), "%04u-%02u-%02uT%02u:%02u:%02uZ", year, month, day, hour, minute, second);

    measurement.pID = userId;
    measurement.weightKg = hasWeight ? weightKg : 0.0f;
    measurement.fatPercentage = bodyFatPercent;
    measurement.musclePercentage = hasMusclePercent ? musclePercent : 0.0f;
    if (hasBodyWaterMass && hasWeight && weightKg > 0.0f) {
        measurement.waterPercentage = (bodyWaterMassKg / weightKg) * 100.0f;
    } else {
        measurement.waterPercentage = 0.0f;
    }

    return true;
}

void storeMeasurement() {
    Serial.printf("personID %d - %s: weight:%4.1fkg, fat:%4.1f%%, water:%4.1f%%, muscle:%4.1f%%\n", measurement.pID, measurement.time, measurement.weightKg, measurement.fatPercentage, measurement.waterPercentage,
                  measurement.musclePercentage);
}

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

float decodeMassKg(uint16_t raw) {
    if (raw == 0) {
        return 0.0f;
    }

    float kg = raw * 0.1f;
    return kg;
}

bool buildMeasurementFromBodyCompositionFrame(const uint8_t *data, size_t length) {
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

    uint16_t raw = 0;

    if (!readUInt16(raw)) {
        return false;
    }
    measurement.fatPercentage = raw / 10.0f;

    uint16_t year = 0;
    uint8_t month = 1;
    uint8_t day = 1;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;

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

    measurement.pID = 0xFF;
    if (offset >= length) {
        Serial.println("Body composition payload truncated (user id)");
        return false;
    }
    measurement.pID = data[offset++];

    if (measurement.pID == 0xFF) {
        Serial.println("Body composition payload missing user id");
        return false;
    }

    // skip basal metabolism
    if (!readUInt16(raw)) {
        return false;
    }

    measurement.musclePercentage = 0.0f;
    raw = 0;
    if (!readUInt16(raw)) {
        return false;
    }
    measurement.musclePercentage = raw / 10.0f;

    float bodyWaterMassKg = 0.0f;
    raw = 0;
    if (!readUInt16(raw)) {
        return false;
    }
    bodyWaterMassKg = decodeMassKg(raw);

    measurement.weightKg = 0.0f;
    raw = 0;
    if (!readUInt16(raw)) {
        return false;
    }
    measurement.weightKg = decodeMassKg(raw);

    snprintf(measurement.time, sizeof(measurement.time), "%04u-%02u-%02uT%02u:%02u:%02uZ", year, month, day, hour, minute, second);

    if (measurement.weightKg > 0.0f) {
        measurement.waterPercentage = (bodyWaterMassKg / measurement.weightKg) * 100.0f;
    } else {
        measurement.waterPercentage = 0.0f;
    }

    return true;
}

void storeMeasurement() {
    Serial.printf("personID %d - %s: weight:%4.1fkg, fat:%4.1f%%, water:%4.1f%%, muscle:%4.1f%%\n", measurement.pID, measurement.time, measurement.weightKg, measurement.fatPercentage, measurement.waterPercentage,
                  measurement.musclePercentage);
}

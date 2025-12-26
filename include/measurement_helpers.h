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

// the data comes in the form: "1e050000e9070c1a12261e020000000000004b03"
// which is hex-encoded bytes
// the first two bytes are flags
// next two bytes are fat percentage (uint16_t, in tenths of percent)
// next 7 bytes are timestamp (year uint16_t, month uint8_t, day

bool buildMeasurementFromBodyCompositionFrame(const uint8_t *data, size_t length) {
    if (length < 4) {
        return false;
    }

    size_t offset = 0;

    // read the next two bytes as uint16_t and advance offset
    auto readUInt16 = [&](uint16_t &value) -> bool {
        if (offset + 2 > length) {
            Serial.println("Body composition payload truncated (uint16)");
            return false;
        }
        value = static_cast<uint16_t>(data[offset]) | (static_cast<uint16_t>(data[offset + 1]) << 8);
        offset += 2;
        return true;
    };

    uint16_t twoByte = 0;

    // skip flags
    if (!readUInt16(twoByte)) {
        return false;
    }

    // fat percentage
    if (!readUInt16(twoByte)) {
        return false;
    }
    measurement.fatPercentage = twoByte / 10.0f;

    // timestamp
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;

    if (offset + 7 > length) {
        Serial.println("Body composition payload truncated (timestamp)");
        return false;
    }

    if (!readUInt16(year)) {
        return false;
    }

    month = data[offset++];
    day = data[offset++];
    hour = data[offset++];
    minute = data[offset++];
    second = data[offset++];

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
    if (!readUInt16(twoByte)) {
        return false;
    }

    measurement.musclePercentage = 0.0f;
    twoByte = 0;
    if (!readUInt16(twoByte)) {
        return false;
    }
    measurement.musclePercentage = twoByte / 10.0f;

    float bodyWaterMassKg = 0.0f;
    twoByte = 0;
    if (!readUInt16(twoByte)) {
        return false;
    }
    bodyWaterMassKg = decodeMassKg(twoByte);

    measurement.weightKg = 0.0f;
    twoByte = 0;
    if (!readUInt16(twoByte)) {
        return false;
    }
    measurement.weightKg = decodeMassKg(twoByte);

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

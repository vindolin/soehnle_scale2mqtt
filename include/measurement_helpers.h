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

#pragma pack(push, 1)
struct BodyCompositionFrame {
    uint16_t flags;
    uint16_t fatPercentage;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t userID;
    uint16_t basalMetabolism;
    uint16_t musclePercentage;
    uint16_t waterMass;
    uint16_t weight;
};
#pragma pack(pop)

bool buildMeasurementFromBodyCompositionFrame(const uint8_t *data, size_t length) {
    if (length < sizeof(BodyCompositionFrame)) {
        Serial.println("Body composition payload truncated");
        return false;
    }

    // map the raw data to the struct
    const BodyCompositionFrame* frame = reinterpret_cast<const BodyCompositionFrame*>(data);

    measurement.fatPercentage = frame->fatPercentage / 10.0f;
    measurement.pID = frame->userID;
    measurement.musclePercentage = frame->musclePercentage / 10.0f;

    float bodyWaterMassKg = decodeMassKg(frame->waterMass);
    measurement.weightKg = decodeMassKg(frame->weight);

    snprintf(measurement.time, sizeof(measurement.time), "%04u-%02u-%02uT%02u:%02u:%02uZ",
             frame->year, frame->month, frame->day, frame->hour, frame->minute, frame->second);

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

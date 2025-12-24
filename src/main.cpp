#include <Arduino.h>
#include <time.h>
#include <WiFi.h>

#include <ArduinoJson.h>
#include <NimBLEDevice.h>
#include <NimBLEScan.h>
#include <PubSubClient.h>

#include "config.h"
#include "users.h"
#include "measurement_utils.h"

constexpr auto BLUE_LED_PIN = 8;

// PWM Configuration
constexpr int PWM_CHANNEL = 0;
constexpr int PWM_FREQ = 5000;
constexpr int PWM_RESOLUTION = 8;
constexpr int MAX_DUTY = 255;
constexpr int DEFAULT_MAX_BRIGHTNESS = (MAX_DUTY * 0.3); // 30% brightness
int currentMaxBrightness = DEFAULT_MAX_BRIGHTNESS;

constexpr auto REQUEST_DELAY_MS = 15000;       // time to wait before requesting history
constexpr auto COLLECT_DELAY_MS = 12000;       // time to wait to collect measurements from notifications
constexpr auto BT_DISCONNECT_DELAY_MS = 45000; // time to wait before the next scan when the last measurement didn't change
// constexpr int SERIAL_STARTUP_DELAY_MS = 4000; // time to wait for Serial to initialize
constexpr auto SERIAL_STARTUP_DELAY_MS = 1000; // time to wait for Serial to initialize

constexpr size_t MQTT_BUFFER_SIZE = 1024;

const auto GMT_OFFSET_SEC = 3600;
const auto DAYLIGHT_OFFSET_SEC = 3600;

const char *BOOT_TIME_TOPIC = "smartscale/bootTime";
const char *BATTERY_LEVEL_TOPIC = "smartscale/battery";

const char *MEASUREMENT_TOPIC = "smartscale/measurement";
const char *MEASUREMENT_TIME_TOPIC = "smartscale/measurementTime";
const char *MEASUREMENT_COUNT_TOPIC = "smartscale/measurementCount";

const char *LOOP_COUNT_TOPIC = "smartscale/loopCount";


Measurement latestMeasurement;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

enum class AppState
{
    SCANNING,
    CONNECTING,
    CONNECTED_WAIT,
    WAIT_FOR_MEASUREMENT,
    // REQUEST_HISTORY,
    // COLLECTING,
    PUBLISHING,
    WAITING_FOR_SCALE_TO_DISAPPEAR
};

AppState currentAppState = AppState::SCANNING;
unsigned long stateTimer = 0;

enum class BlinkState
{
    OFF,
    ON
};

enum class LedMode
{
    PULSE,
    OFF,
};

uint16_t blinkOnDurationMs = 200;
uint16_t blinkOffDurationMs = 800;

LedMode currentLedMode = LedMode::OFF;
unsigned long lastBlinkToggle = 0;

NimBLEClient *pClient = nullptr;

const char *SCALE_DEVICE_NAME = "Shape100";

// --- UUIDs ---
static NimBLEUUID SVC_BATTERY("180f");
static NimBLEUUID CHR_BATTERY_LEVEL("2a19");

static NimBLEUUID SVC_CURRENT_TIME("1805");
static NimBLEUUID CHR_CURRENT_TIME("2a2b");

static NimBLEUUID SVC_USER_DATA("181c");
static NimBLEUUID CHR_USER_CONTROL_POINT("2a9f");

static NimBLEUUID SVC_SOEHNLE("352e3000-28e9-40b8-a361-6db4cca4147c");
static NimBLEUUID CHR_MEASUREMENT_NOTIFY("352e3001-28e9-40b8-a361-6db4cca4147c");
static NimBLEUUID CHR_REQUEST_HISTORY("352e3002-28e9-40b8-a361-6db4cca4147c");

static NimBLEUUID SVC_WEIGHT("0000181d-0000-1000-8000-00805f9b34fb");
static NimBLEUUID CHR_WEIGHT_INDICATE("00002a9d-0000-1000-8000-00805f9b34fb");
static NimBLEUUID CHR_WEIGHT("00002a9e-0000-1000-8000-00805f9b34fb");

static NimBLEUUID SVC_BODY_COMPOSITION("0000181b-0000-1000-8000-00805f9b34fb");
static NimBLEUUID CHR_BODY_COMPOSITION_MEASUREMENT("00002a9c-0000-1000-8000-00805f9b34fb");

NimBLEAdvertisedDevice *scaleDevice = nullptr;

uint8_t batteryLevel = 0;

Measurement lastPublishedMeasurement;

void setLed(bool on)
{
    // Active LOW: 0 is ON (Max brightness), 255 is OFF.
    int duty = on ? (MAX_DUTY - currentMaxBrightness) : MAX_DUTY;
    ledcWrite(PWM_CHANNEL, duty);
}

bool ledState = false;

void toggleLed()
{
    // static bool ledState = false;
    ledState = !ledState;
    setLed(ledState);
}

uint16_t measurementCount = 0;
uint32_t loopCount = 0;

void logHexPayload(const uint8_t *data, size_t length)
{
    Serial.print("Received measurement data: ");
    for (size_t i = 0; i < length; i++)
    {
        Serial.printf("%02x", data[i]);
    }
    Serial.println();
}

namespace
{
constexpr uint16_t BODY_COMP_FLAG_UNITS_IMPERIAL = 0x0001;
constexpr uint16_t BODY_COMP_FLAG_TIMESTAMP_PRESENT = 0x0002;
constexpr uint16_t BODY_COMP_FLAG_USER_ID_PRESENT = 0x0004;
constexpr uint16_t BODY_COMP_FLAG_BASAL_METABOLISM = 0x0008;
constexpr uint16_t BODY_COMP_FLAG_MUSCLE_PERCENT = 0x0010;
constexpr uint16_t BODY_COMP_FLAG_MUSCLE_MASS = 0x0020;
constexpr uint16_t BODY_COMP_FLAG_FAT_FREE_MASS = 0x0040;
constexpr uint16_t BODY_COMP_FLAG_SOFT_LEAN_MASS = 0x0080;
constexpr uint16_t BODY_COMP_FLAG_BODY_WATER_MASS = 0x0100;
constexpr uint16_t BODY_COMP_FLAG_IMPEDANCE = 0x0200;
constexpr uint16_t BODY_COMP_FLAG_WEIGHT = 0x0400;
constexpr uint16_t BODY_COMP_FLAG_HEIGHT = 0x0800;
constexpr uint16_t BODY_COMP_FLAG_MULTI_PACKET = 0x1000;

float decodeMassKg(uint16_t raw, bool usesImperialUnits)
{
    if (raw == 0)
    {
        return 0.0f;
    }

    if (usesImperialUnits)
    {
        return raw * 0.01f * 0.45359237f;
    }

    float kg = raw * 0.005f;
    if (kg < 10.0f)
    {
        // Soehnle appears to use 0.1 kg resolution despite the spec stating 0.005 kg.
        kg = raw * 0.1f;
    }
    return kg;
}

bool isLegacyMeasurementFrame(const uint8_t *data, size_t length)
{
    return length == MEASUREMENT_FRAME_LENGTH && data[0] == MEASUREMENT_OPCODE;
}

bool buildMeasurementFromLegacyFrame(const uint8_t *data, size_t length, Measurement &outMeasurement)
{
    MeasurementFrame frame;
    if (!parseMeasurementFrame(data, length, frame))
    {
        Serial.println("Skipping invalid measurement frame");
        return false;
    }

    auto userIt = users.find(frame.pID);
    if (userIt == users.end())
    {
        Serial.printf("Unknown user pID: %d\n", frame.pID);
        return false;
    }
    const User &user = userIt->second;

    auto fat = 0.0f;
    auto water = 0.0f;
    auto muscle = 0.0f;
    if (frame.imp50 > 0)
    {
        fat = calculateFat(user, frame.weightKg, frame.imp50);
        water = calculateWater(user, frame.weightKg, frame.imp50);
        muscle = calculateMuscle(user, frame.weightKg, frame.imp50, frame.imp5);
    }

    char timeStr[25];
    snprintf(timeStr, sizeof(timeStr), "%04d-%02d-%02dT%02d:%02d:%02dZ",
             frame.year, frame.month, frame.day, frame.hour, frame.minute, frame.second);

    outMeasurement.pID = frame.pID;
    outMeasurement.time = timeStr;
    outMeasurement.weight = frame.weightKg;
    outMeasurement.fat = fat;
    outMeasurement.water = water;
    outMeasurement.muscle = muscle;
    return true;
}

bool buildMeasurementFromBodyCompositionFrame(const uint8_t *data, size_t length, Measurement &outMeasurement)
{
    if (length < 4)
    {
        return false;
    }

    size_t offset = 0;
    auto readUInt16 = [&](uint16_t &value) -> bool {
        if (offset + 2 > length)
        {
            Serial.println("Body composition payload truncated (uint16)");
            return false;
        }
        value = static_cast<uint16_t>(data[offset]) | (static_cast<uint16_t>(data[offset + 1]) << 8);
        offset += 2;
        return true;
    };

    const uint16_t flags = static_cast<uint16_t>(data[offset]) | (static_cast<uint16_t>(data[offset + 1]) << 8);
    offset += 2;

    if (flags & BODY_COMP_FLAG_MULTI_PACKET)
    {
        Serial.println("Multiple packet body composition measurements are not supported");
        return false;
    }

    uint16_t bodyFatRaw = 0;
    if (!readUInt16(bodyFatRaw))
    {
        return false;
    }
    float bodyFatPercent = bodyFatRaw / 10.0f;

    uint16_t year = 0;
    uint8_t month = 1;
    uint8_t day = 1;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    if (flags & BODY_COMP_FLAG_TIMESTAMP_PRESENT)
    {
        if (offset + 7 > length)
        {
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
    else
    {
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
    if (flags & BODY_COMP_FLAG_USER_ID_PRESENT)
    {
        if (offset >= length)
        {
            Serial.println("Body composition payload truncated (user id)");
            return false;
        }
        userId = data[offset++];
    }

    if (userId == 0xFF)
    {
        Serial.println("Body composition payload missing user id");
        return false;
    }

    auto userIt = users.find(userId);
    if (userIt == users.end())
    {
        Serial.printf("Unknown user pID: %d\n", userId);
        return false;
    }

    if (flags & BODY_COMP_FLAG_BASAL_METABOLISM)
    {
        uint16_t basalRaw = 0;
        if (!readUInt16(basalRaw))
        {
            return false;
        }
        // Basal metabolism is currently unused but kept for completeness.
    }

    float musclePercent = 0.0f;
    bool hasMusclePercent = false;
    if (flags & BODY_COMP_FLAG_MUSCLE_PERCENT)
    {
        uint16_t raw = 0;
        if (!readUInt16(raw))
        {
            return false;
        }
        musclePercent = raw / 10.0f;
        hasMusclePercent = true;
    }

    if (flags & BODY_COMP_FLAG_MUSCLE_MASS)
    {
        uint16_t skip = 0;
        if (!readUInt16(skip))
        {
            return false;
        }
    }

    if (flags & BODY_COMP_FLAG_FAT_FREE_MASS)
    {
        uint16_t skip = 0;
        if (!readUInt16(skip))
        {
            return false;
        }
    }

    if (flags & BODY_COMP_FLAG_SOFT_LEAN_MASS)
    {
        uint16_t skip = 0;
        if (!readUInt16(skip))
        {
            return false;
        }
    }

    const bool usesImperialUnits = (flags & BODY_COMP_FLAG_UNITS_IMPERIAL) != 0;

    float bodyWaterMassKg = 0.0f;
    bool hasBodyWaterMass = false;
    if (flags & BODY_COMP_FLAG_BODY_WATER_MASS)
    {
        uint16_t raw = 0;
        if (!readUInt16(raw))
        {
            return false;
        }
        bodyWaterMassKg = decodeMassKg(raw, usesImperialUnits);
        hasBodyWaterMass = true;
    }

    float impedanceOhms = 0.0f;
    if (flags & BODY_COMP_FLAG_IMPEDANCE)
    {
        uint16_t raw = 0;
        if (!readUInt16(raw))
        {
            return false;
        }
        impedanceOhms = raw / 10.0f;
    }

    float weightKg = 0.0f;
    bool hasWeight = false;
    if (flags & BODY_COMP_FLAG_WEIGHT)
    {
        uint16_t raw = 0;
        if (!readUInt16(raw))
        {
            return false;
        }
        weightKg = decodeMassKg(raw, usesImperialUnits);
        hasWeight = true;
    }

    if (flags & BODY_COMP_FLAG_HEIGHT)
    {
        uint16_t skip = 0;
        if (!readUInt16(skip))
        {
            return false;
        }
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
    if (hasBodyWaterMass && hasWeight && weightKg > 0.0f)
    {
        outMeasurement.water = (bodyWaterMassKg / weightKg) * 100.0f;
    }
    else
    {
        outMeasurement.water = 0.0f;
    }

    return true;
}

void logAndStoreMeasurement(const Measurement &measurement)
{
    Serial.printf("personID %d - %s: weight:%4.1fkg, fat:%4.1f%%, water:%4.1f%%, muscle:%4.1f%%\n",
                  measurement.pID,
                  measurement.time.c_str(),
                  measurement.weight,
                  measurement.fat,
                  measurement.water,
                  measurement.muscle);

    if (latestMeasurement.time.isEmpty() || measurement.time > latestMeasurement.time)
    {
        latestMeasurement = measurement;
    }
}

} // namespace

void processMeasurementPayload(const uint8_t *data, size_t length)
{
    measurementCount++;

    Measurement measurement;

    if (isLegacyMeasurementFrame(data, length))
    {
        if (buildMeasurementFromLegacyFrame(data, length, measurement))
        {
            logAndStoreMeasurement(measurement);
        }
        return;
    }

    if (buildMeasurementFromBodyCompositionFrame(data, length, measurement))
    {
        logAndStoreMeasurement(measurement);
        return;
    }

    Serial.println("Skipping measurement payload: unsupported format");
}

void indicateWeight(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    Serial.println("Weight indication received:");
    logHexPayload(pData, length);
    // processMeasurementPayload(pData, length);
}

void indicateBodyComposition(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    Serial.println("Body Composition indication received:");
    logHexPayload(pData, length);
    processMeasurementPayload(pData, length);
}

bool connectToScaleDevice()
{
    Serial.printf("Connecting to %s\n", scaleDevice->getAddress().toString().c_str());

    pClient = NimBLEDevice::createClient();
    // pClient->setClientCallbacks(new ClientCallbacks());
    // pClient->setClientCallbacks(&clientCallbacks); // this leads to a crash

    if (!pClient->connect(scaleDevice))
    {
        Serial.println("Failed to connect");
        NimBLEDevice::deleteClient(pClient);
        pClient = nullptr;
        return false;
    }

    // Battery Service
    NimBLERemoteService *pSvcBattery = pClient->getService(SVC_BATTERY);
    if (pSvcBattery)
    {
        NimBLERemoteCharacteristic *pChrBattery = pSvcBattery->getCharacteristic(CHR_BATTERY_LEVEL);
        if (pChrBattery)
        {
            if (pChrBattery->canRead())
            {
                std::string value = pChrBattery->readValue();
                if (value.length() > 0)
                {
                    batteryLevel = value[0];
                    Serial.printf("Initial Battery: %d%%\n", batteryLevel);
                }
            }
        }
    }

    // Current Time Service
    NimBLERemoteService *pSvcTime = pClient->getService(SVC_CURRENT_TIME);
    if (pSvcTime)
    {
        NimBLERemoteCharacteristic *pChrTime = pSvcTime->getCharacteristic(CHR_CURRENT_TIME);
        Serial.println(pChrTime->getValue()); // just to check if it exists

        if (pChrTime && pChrTime->canWrite())
        {
            struct tm timeinfo;
            if (getLocalTime(&timeinfo))
            {
                uint16_t year = timeinfo.tm_year + 1900;
                uint8_t month = timeinfo.tm_mon + 1;
                uint8_t day = timeinfo.tm_mday;
                uint8_t hour = timeinfo.tm_hour;
                uint8_t minute = timeinfo.tm_min;
                uint8_t second = timeinfo.tm_sec;
                // tm_wday: 0=Sunday, 1=Monday...
                // Scale expects: 1=Monday... 7=Sunday
                uint8_t weekday = (timeinfo.tm_wday == 0) ? 7 : timeinfo.tm_wday;

                uint8_t timeData[10];
                timeData[0] = year & 0xFF;
                timeData[1] = (year >> 8) & 0xFF;
                timeData[2] = month;
                timeData[3] = day;
                timeData[4] = hour;
                timeData[5] = minute;
                timeData[6] = second;
                timeData[7] = weekday;
                timeData[8] = 0;
                timeData[9] = 0;

                pChrTime->writeValue(timeData, 10, true);
                Serial.printf("Time set to: %04d-%02d-%02d %02d:%02d:%02d\n", year, month, day, hour, minute, second);
            }
            else
            {
                Serial.println("Time not set (no local time)");
            }
        }
    }

    NimBLERemoteService* pSvcWeight = pClient->getService(SVC_WEIGHT);
    if (pSvcWeight) {
        Serial.println("Found Weight Service");
        // register indicate for weight
        NimBLERemoteCharacteristic* pChrWeight = pSvcWeight->getCharacteristic(CHR_WEIGHT_INDICATE);
        if (pChrWeight && pChrWeight->canIndicate()) {
            if(pChrWeight->subscribe(false, indicateWeight)) {
                Serial.println("Subscribed to weight...");
            } else {
                Serial.println("Failed to subscribe to weight!");
            }
        }
    }

    NimBLERemoteService* pSvcBodyComposition = pClient->getService(SVC_BODY_COMPOSITION);
    if (pSvcBodyComposition) {
        Serial.println("Found Body Composition Service");
        NimBLERemoteCharacteristic* pChrBodyComposition = pSvcBodyComposition->getCharacteristic(CHR_BODY_COMPOSITION_MEASUREMENT);
        if (pChrBodyComposition && pChrBodyComposition->canIndicate()) {
            if (pChrBodyComposition->subscribe(false, indicateBodyComposition)) {
                Serial.println("Subscribed to body composition measurement...");
            } else {
                Serial.println("Failed to subscribe to body composition measurement!");
            }
        }
    }

    return true;
}

bool disconnectFromScaleDevice()
{
    if (pClient && pClient->isConnected())
    {
        pClient->disconnect();
        return true;
    }
    return false;
}

class BLEScanCallbacks : public NimBLEScanCallbacks
{
    void onResult(const NimBLEAdvertisedDevice *advertisedDevice)
    {
        if (advertisedDevice->haveName())
        {
            String name = advertisedDevice->getName().c_str();
            String mac = advertisedDevice->getAddress().toString().c_str();

            Serial.printf("Device: %s @ %s\n", name.c_str(), mac.c_str());

            if (name.indexOf(SCALE_DEVICE_NAME) >= 0)
            {
                Serial.printf("Found Scale: %s @ %s\n", name.c_str(), mac.c_str());
                NimBLEDevice::getScan()->stop();
                scaleDevice = new NimBLEAdvertisedDevice(*advertisedDevice);
                return;
            }
        }
    }
};

static BLEScanCallbacks scanCallbacks;

void connectToMqtt()
{
    uint connectAttempts = 0;
    while (!mqttClient.connected())
    {
        if (connectAttempts > 5)
        {
            ESP.restart();
        }
        connectAttempts++;

        Serial.println("Attempting MQTT connection...");
        if (mqttClient.connect("ESP32ScaleClientX", MQTT_SERVER_USER, MQTT_SERVER_PASSWORD))
        {
            Serial.println("MQTT connected");
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 2 seconds");
            delay(2000);
        }
    }
}

void disconnectFromMqtt()
{
    mqttClient.disconnect();
    Serial.println("MQTT disconnected");
}

void connectToWifi()
{
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    uint connectAttempts = 0;
    while (WiFi.status() != WL_CONNECTED)
    {
        if (connectAttempts > 5)
        {
            ESP.restart();
        }
        connectAttempts++;

        delay(1000);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void disconnectFromWifi()
{
    WiFi.disconnect(true);
    Serial.println("WiFi disconnected");
}

void startScan()
{
    NimBLEScan *pBLEScan = NimBLEDevice::getScan();
    // pBLEScan->setScanCallbacks(new BLEScanCallbacks());
    pBLEScan->setScanCallbacks(&scanCallbacks);
    if (pBLEScan->start(0, false))
    {
        Serial.println("BLE scan started successfully, waiting for scale device...");
    }
    else
    {
        Serial.println("Failed to start scan");
    }
}

void requestHistoryForAllUsers()
{
    measurementCount = 0;
    if (pClient == nullptr)
        return;

    NimBLERemoteService *pSvcSoehnle = pClient->getService(SVC_SOEHNLE);
    if (pSvcSoehnle)
    {
        NimBLERemoteCharacteristic *historyRequestCharacteristic = pSvcSoehnle->getCharacteristic(CHR_REQUEST_HISTORY);
        Serial.println("Requesting history for all users...");
        if (historyRequestCharacteristic)
        {
            for (int i = 1; i <= userCount; i++)
            {
                uint8_t cmd[] = {MEASUREMENT_OPCODE, (uint8_t)i};
                historyRequestCharacteristic->writeValue(cmd, 2, true);
                delay(500);
            }
        }
    }
}

String buildCurrentTimeString()
{
    time_t now = time(nullptr);
    struct tm timeinfo;
    if (localtime_r(&now, &timeinfo) != nullptr)
    {
        char buffer[25];
        strftime(buffer, sizeof(buffer), "%d.%m.%Y - %H:%M:%S ", &timeinfo);
        return String(buffer);
    }
    return String(now);
}

void syncTime()
{
    connectToWifi();
    delay(100);

    // Sync time via NTP
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
    struct tm timeinfo;
    if (getLocalTime(&timeinfo))
    {
        connectToMqtt();
        mqttClient.publish(BOOT_TIME_TOPIC, buildCurrentTimeString().c_str(), true);
        mqttClient.loop();
        delay(1000); // wait for publish
        disconnectFromMqtt();

        Serial.println(&timeinfo, "Time synced: %A, %B %d %Y %H:%M:%S");
    }
    else
    {
        Serial.println("Failed to sync time");
    }
    disconnectFromWifi();
}

void checkRestart()
{
    // restart the ESP at 01:00 every day to avoid memory leaks
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    if (timeinfo.tm_hour == 1 && timeinfo.tm_min == 0)
    {
        Serial.println("Restarting ESP32 to avoid memory leaks...");
        delay(5000); // Wait a bit before restarting
        ESP.restart();
    }
}

bool generateMeasurementJson(String &outJson)
{
    if (latestMeasurement.time.isEmpty())
    {
        return false;
    }
    DynamicJsonDocument doc(512);

    doc["p_id"] = latestMeasurement.pID;
    doc["time"] = latestMeasurement.time;
    doc["weight"] = latestMeasurement.weight;
    doc["fat"] = latestMeasurement.fat;
    doc["water"] = latestMeasurement.water;
    doc["muscle"] = latestMeasurement.muscle;
    serializeJson(doc, outJson);
    return true;
}

void cleanupBleSession()
{
    if (pClient != nullptr)
    {
        if (pClient->isConnected())
        {
            pClient->disconnect();
        }
        NimBLEDevice::deleteClient(pClient);
        pClient = nullptr;
    }
    if (scaleDevice != nullptr)
    {
        delete scaleDevice;
        scaleDevice = nullptr;
    }

    // disable BLE to free memory
    // NimBLEDevice::deinit();
}

void setup()
{
    Serial.begin(115200);
    delay(SERIAL_STARTUP_DELAY_MS); // Wait for CDC Serial to initialize

    // Configure PWM
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(BLUE_LED_PIN, PWM_CHANNEL);
    setLed(false);

    mqttClient.setBufferSize(MQTT_BUFFER_SIZE);
    mqttClient.setServer(MQTT_SERVER_IP, MQTT_SERVER_PORT);

    syncTime();

    NimBLEDevice::init("ESP32_SCALE");
    NimBLEDevice::setPower(ESP_PWR_LVL_P21); // max power
    NimBLEDevice::setSecurityAuth(true, true, true);
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
}

void setLedBlinkDurations(uint16_t onDurationMs, uint16_t offDurationMs)
{
    blinkOnDurationMs = onDurationMs;
    blinkOffDurationMs = offDurationMs;
}

void setLedModeBlink(uint16_t onDurationMs = 200, uint16_t offDurationMs = 800)
{
    currentLedMode = LedMode::PULSE;
    currentMaxBrightness = DEFAULT_MAX_BRIGHTNESS;
    setLedBlinkDurations(onDurationMs, offDurationMs);
}

void setLedModeOff()
{
    currentLedMode = LedMode::OFF;
    setLed(false);
}

void loop()
{
    checkRestart();

    switch (currentAppState)
    {
    case AppState::SCANNING:
        if (scaleDevice != nullptr)
        {
            currentAppState = AppState::CONNECTING;
        }
        else
        {
            NimBLEScan *pScan = NimBLEDevice::getScan();
            if (!pScan->isScanning())
            {
                setLedModeBlink(100, 2000);
                startScan();
            }
        }
        break;

    case AppState::CONNECTING:
        loopCount++;

        setLedModeBlink(50, 100);

        latestMeasurement = Measurement(); // Clear previous measurement
        if (connectToScaleDevice())
        {
            setLedModeBlink(100, 100);
            currentAppState = AppState::CONNECTED_WAIT;
            stateTimer = millis();
        }
        else
        {
            Serial.println("Failed to connect, restarting scan...");
            if (scaleDevice != nullptr)
            {
                delete scaleDevice;
                scaleDevice = nullptr;
            }
            currentAppState = AppState::SCANNING;
        }
        break;

    case AppState::CONNECTED_WAIT:
        if (!pClient || !pClient->isConnected())
        {
            Serial.println("Lost connection during wait!");
            cleanupBleSession();
            currentAppState = AppState::SCANNING;
            break;
        }
        if (millis() - stateTimer > REQUEST_DELAY_MS)
        {
            // currentAppState = AppState::REQUEST_HISTORY;
            currentAppState = AppState::WAIT_FOR_MEASUREMENT;
        }
        break;

        case AppState::WAIT_FOR_MEASUREMENT:
        if (!pClient || !pClient->isConnected())
        {
            Serial.println("Lost connection while waiting for measurement!");
            cleanupBleSession();
            currentAppState = AppState::SCANNING;
            break;
        }

    case AppState::WAITING_FOR_SCALE_TO_DISAPPEAR:
        currentMaxBrightness = map(millis() - stateTimer, 0, BT_DISCONNECT_DELAY_MS, DEFAULT_MAX_BRIGHTNESS, 0);
        currentMaxBrightness = constrain(currentMaxBrightness, 0, DEFAULT_MAX_BRIGHTNESS);

        if (millis() - stateTimer > BT_DISCONNECT_DELAY_MS)
        {
            currentAppState = AppState::SCANNING;
        }
        break;

    } // end switch app state

    switch (currentLedMode)
    {
    case LedMode::PULSE:
    {
        unsigned long period = blinkOnDurationMs + blinkOffDurationMs;
        if (period == 0)
            period = 1000;

        float phase = (float)(millis() % period) / period * 2 * PI;
        float val = (1.0f - cos(phase)) / 2.0f;

        int brightness = val * currentMaxBrightness;
        int duty = MAX_DUTY - brightness; // Active LOW
        ledcWrite(PWM_CHANNEL, duty);
    }
    break;
    case LedMode::OFF:
        break;
    } // end switch led mode
}

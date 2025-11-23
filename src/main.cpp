#include <Arduino.h>
#include <map>
#include <time.h>
#include <vector>
#include <WiFi.h>

#include "config.h"

#include <ArduinoJson.h>
#include <NimBLEDevice.h>
#include <NimBLEScan.h>
#include <PubSubClient.h>

struct User {
    int age;
    float height;
    bool is_male;
    int activity_level;
};

struct Measurement {
    String user;
    String time;
    float weight;
    float fat;
    float water;
    float muscle;
};

struct MeasurementFrame {
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

std::vector<Measurement> collectedMeasurements;

constexpr int restartDelayMs = 45000;
constexpr int requestDelayMs = 5000;
constexpr size_t mqttBufferSize = 1024;
constexpr size_t measurementFrameLength = 15;
constexpr uint8_t measurementOpcode = 0x09;

const char* SCALE_DEVICE_NAME = "Shape100";

const char* ntpServer = "fritz.box";

const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;

const char* dataPublishTopic = "smartscale/data";
const char* batteryLevelTopic = "smartscale/battery";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

std::map<String, User> users = {
    // name, age, height, is_male, activity_level (1-5)
    {"mona", {50, 159.0, false, 2}},
    {"tulpe", {55, 180.0, true, 2}}
};

struct UserDetectionRule {
    String name;
    float minWeightKg;
    float maxWeightKg;
};

std::vector<UserDetectionRule> detectionRules = {
    {"mona", 45.0f, 74.9f},
    {"tulpe", 75.0f, 89.0f}
};

uint8_t userCount = static_cast<uint8_t>(detectionRules.size());

bool isScanning = false;
bool scaleConnected = false;
bool taskCompleted = false;
bool historyRequested = false;
unsigned long connectionTime = 0;
NimBLEClient* pClient = nullptr;

// --- UUIDs ---
static NimBLEUUID SVC_BATTERY("180f");
static NimBLEUUID CHR_BATTERY_LEVEL("2a19");

static NimBLEUUID SVC_CURRENT_TIME("1805");
static NimBLEUUID CHR_CURRENT_TIME("2a2b");

static NimBLEUUID SVC_USER_DATA("181c");
static NimBLEUUID CHR_USER_CONTROL_POINT("2a9f");

static NimBLEUUID SVC_SOEHNLE("352e3000-28e9-40b8-a361-6db4cca4147c");
static NimBLEUUID CHR_SOEHNLE_A("352e3001-28e9-40b8-a361-6db4cca4147c"); // Measurements
static NimBLEUUID CHR_SOEHNLE_CMD("352e3002-28e9-40b8-a361-6db4cca4147c"); // Commands

NimBLEAdvertisedDevice* scaleDevice = nullptr;

uint8_t batteryLevel = 0;

// https://github.com/oliexdev/openScale/blob/master/android_app/app/src/main/java/com/health/openscale/core/bluetooth/scales/SoehnleHandler.kt
float get_fat(const User& user, float weight, float imp50) {
    float activity_corr_fac = 0.0;
    if (user.activity_level == 4) activity_corr_fac = user.is_male ? 2.5 : 2.3;
    else if (user.activity_level == 5) activity_corr_fac = user.is_male ? 4.3 : 4.1;

    float sex_corr_fac = user.is_male ? 0.250 : 0.214;
    float activity_sex_div = user.is_male ? 65.5 : 55.1;

    return (1.847 * weight * 10000.0 / (user.height * user.height) + 
            sex_corr_fac * user.age + 0.062 * imp50 - 
            (activity_sex_div - activity_corr_fac));
}

float get_water(const User& user, float weight, float imp50) {
    float activity_corr_fac = 0.0;
    if (user.activity_level >= 1 && user.activity_level <= 3) activity_corr_fac = user.is_male ? 2.83 : 0.0;
    else if (user.activity_level == 4) activity_corr_fac = user.is_male ? 3.93 : 0.4;
    else if (user.activity_level == 5) activity_corr_fac = user.is_male ? 5.33 : 1.4;

    return ((0.3674 * user.height * user.height / imp50 + 
            0.17530 * weight - 0.11 * user.age + 
            (6.53 + activity_corr_fac)) / weight * 100.0);
}

float get_muscle(const User& user, float weight, float imp50, float imp5) {
    float activity_corr_fac = 0.0;
    if (user.activity_level >= 1 && user.activity_level <= 3) activity_corr_fac = user.is_male ? 3.6224 : 0.0;
    else if (user.activity_level == 4) activity_corr_fac = user.is_male ? 4.3904 : 0.0;
    else if (user.activity_level == 5) activity_corr_fac = user.is_male ? 5.4144 : 1.664;

    return (((0.47027 / imp50 - 0.24196 / imp5) * user.height * user.height + 
            0.13796 * weight - 0.1152 * user.age + 
            (5.12 + activity_corr_fac)) / weight * 100.0);
}

void logHexPayload(const uint8_t* data, size_t length) {
    Serial.print("Received measurement data: ");
    for (size_t i = 0; i < length; i++) {
        Serial.printf("%02x", data[i]);
    }
    Serial.println();
}

bool parseMeasurementFrame(const uint8_t* data, size_t length, MeasurementFrame& frame) {
    if (length != measurementFrameLength || data[0] != measurementOpcode) {
        return false;
    }

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

const User* resolveUserByWeight(float weightKg, String& userName) {
    for (const auto& rule : detectionRules) {
        if (weightKg >= rule.minWeightKg && weightKg <= rule.maxWeightKg) {
            auto it = users.find(rule.name);
            if (it != users.end()) {
                userName = rule.name;
                return &it->second;
            }
        }
    }
    return nullptr;
}

void notifyMeasurement(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    // logHexPayload(pData, length);

    MeasurementFrame frame;
    if (!parseMeasurementFrame(pData, length, frame)) {
        Serial.println("Skipping invalid measurement frame");
        return;
    }

    String userName;
    const User* user = resolveUserByWeight(frame.weightKg, userName);
    if (user == nullptr) {
        Serial.println("Unknown user based on weight");
        return;
    }

    float fat = get_fat(*user, frame.weightKg, frame.imp50);
    float water = get_water(*user, frame.weightKg, frame.imp50);
    float muscle = get_muscle(*user, frame.weightKg, frame.imp50, frame.imp5);

    char timeStr[25];
    snprintf(timeStr, sizeof(timeStr), "%04d-%02d-%02dT%02d:%02d:%02dZ", frame.year, frame.month, frame.day, frame.hour, frame.minute, frame.second);

    Measurement m;
    m.user = userName;
    m.time = String(timeStr);
    m.weight = frame.weightKg;
    m.fat = fat;
    m.water = water;
    m.muscle = muscle;
    collectedMeasurements.push_back(m);
}

void notifyBattery(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    if (length > 0) {
        Serial.printf("Battery level: %d%%\n", pData[0]);
    }
}

class ClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pClient) {
        Serial.println("Connected to scale");
        scaleConnected = true;
        historyRequested = false;
        connectionTime = millis();
    }

    void onDisconnect(NimBLEClient* pClient) {
        Serial.println("Disconnected");
        scaleConnected = false;
        // We don't delete the client here, we'll handle it in the loop
    }
};

bool connectToScaleDevice() {
    Serial.printf("Connecting to %s\n", scaleDevice->getAddress().toString().c_str());

    pClient = NimBLEDevice::createClient();
    pClient->setClientCallbacks(new ClientCallbacks());

    if (!pClient->connect(scaleDevice)) {
        Serial.println("Failed to connect");
        return false;
    }

    // Battery Service
    NimBLERemoteService* pSvcBattery = pClient->getService(SVC_BATTERY);
    if (pSvcBattery) {
        NimBLERemoteCharacteristic* pChrBattery = pSvcBattery->getCharacteristic(CHR_BATTERY_LEVEL);
        if (pChrBattery) {
            if(pChrBattery->canRead()) {
                std::string value = pChrBattery->readValue();
                if (value.length() > 0) {
                    batteryLevel = value[0];
                    Serial.printf("Initial Battery: %d%%\n", batteryLevel);
                }
            }
        }
    }

    // Current Time Service
    NimBLERemoteService* pSvcTime = pClient->getService(SVC_CURRENT_TIME);
    if (pSvcTime) {
        NimBLERemoteCharacteristic* pChrTime = pSvcTime->getCharacteristic(CHR_CURRENT_TIME);
        if (pChrTime && pChrTime->canWrite()) {
            struct tm timeinfo;
            if (getLocalTime(&timeinfo)) {
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
            } else {
                Serial.println("Time not set (no local time)");
            }
        }
    }

    // Soehnle Service
    NimBLERemoteService* pSvcSoehnle = pClient->getService(SVC_SOEHNLE);
    if (pSvcSoehnle) {

        // register notify for measurements
        NimBLERemoteCharacteristic* pChrSoehnleA = pSvcSoehnle->getCharacteristic(CHR_SOEHNLE_A);
        if (pChrSoehnleA && pChrSoehnleA->canNotify()) {
            pChrSoehnleA->subscribe(true, notifyMeasurement);
            Serial.println("Subscribed to measurements");
        }
        
        // History request moved to loop
    }

    return true;
}

bool disconnectFromScaleDevice() {
    if (pClient && pClient->isConnected()) {
        pClient->disconnect();
        return true;
    }
    return false;
}

class MyScanCallbacks: public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice* advertisedDevice) {
        if (advertisedDevice->haveName()) {
            String name = advertisedDevice->getName().c_str();
            String mac = advertisedDevice->getAddress().toString().c_str();
            if (name.indexOf(SCALE_DEVICE_NAME) >= 0) {
                Serial.printf("Found Scale: %s @ %s\n", name.c_str(), mac.c_str());
                NimBLEDevice::getScan()->stop();
                scaleDevice = new NimBLEAdvertisedDevice(*advertisedDevice);
                isScanning = false;
                return;
            }
        }
    }
};

void connectToMqtt() {
    while (!mqttClient.connected()) {
        Serial.println("Attempting MQTT connection...");
        if (mqttClient.connect("ESP32ScaleClient", MQTT_SERVER_USER, MQTT_SERVER_PASSWORD)) {
            Serial.println("MQTT connected");
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void disconnectFromMqtt() {
    mqttClient.disconnect();
    Serial.println("MQTT disconnected");
}

void connectToWifi() {
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void disconnectFromWifi() {
    WiFi.disconnect(true);
    Serial.println("WiFi disconnected");
}

void startScan() {
    NimBLEScan* pBLEScan = NimBLEDevice::getScan();
    pBLEScan->setScanCallbacks(new MyScanCallbacks());
    // pBLEScan->setInterval(45);
    // pBLEScan->setWindow(15);
    // pBLEScan->setActiveScan(true);
    if(pBLEScan->start(0, false)) {
        isScanning = true;
        Serial.println("BLE scan started successfully");
    } else {
        Serial.println("Failed to start scan");
        isScanning = false;
    }
}

void requestHistoryForAllUsers() {
    historyRequested = true;
    if (pClient == nullptr) return;

    NimBLERemoteService* pSvcSoehnle = pClient->getService(SVC_SOEHNLE);
    if (pSvcSoehnle) {
        NimBLERemoteCharacteristic* pChrSoehnleCmd = pSvcSoehnle->getCharacteristic(CHR_SOEHNLE_CMD);
        if (pChrSoehnleCmd) {
             Serial.println("Requesting history for all users...");
             for (int i = 1; i <= userCount; i++) {
                 uint8_t cmd[] = {0x09, (uint8_t)i};
                 pChrSoehnleCmd->writeValue(cmd, 2, true);
                 delay(100);
             }
        }
    }
}

void syncTime() {
    connectToWifi();

    // Sync time via NTP
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
        Serial.println(&timeinfo, "Time synced: %A, %B %d %Y %H:%M:%S");
    } else {
        Serial.println("Failed to sync time");
    }
    disconnectFromWifi();
}

void setup() {
    Serial.begin(115200);
    delay(4000);  // Wait for CDC Serial to initialize

    mqttClient.setBufferSize(mqttBufferSize);
    mqttClient.setServer(MQTT_SERVER_IP, MQTT_SERVER_PORT);

    syncTime();
    
    NimBLEDevice::init("ESP32_SCALE");
    startScan();
    Serial.println("Scan started...");
}

void checkRestart() {
    // restart the ESP at 01:00 every day to avoid memory leaks
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    if (timeinfo.tm_hour == 1 && timeinfo.tm_min == 0) {
        Serial.println("Restarting ESP32 to avoid memory leaks...");
        delay(5000); // Wait a bit before restarting
        ESP.restart();
    }
}

bool buildLatestMeasurementJson(String& outJson) {
    DynamicJsonDocument doc(1024);

    if (!collectedMeasurements.empty()) {
        const auto latestIt = std::max_element(
            collectedMeasurements.begin(),
            collectedMeasurements.end(),
            [](const Measurement& lhs, const Measurement& rhs) {
                return lhs.time < rhs.time;
            });

        doc["user"] = latestIt->user;
        doc["time"] = latestIt->time;
        doc["weight"] = latestIt->weight;
        doc["fat"] = latestIt->fat;
        doc["water"] = latestIt->water;
        doc["muscle"] = latestIt->muscle;
    }

    serializeJson(doc, outJson);
    return !collectedMeasurements.empty();
}

void cleanupBleSession() {
    if (pClient != nullptr) {
        if (pClient->isConnected()) {
            pClient->disconnect();
        }
        NimBLEDevice::deleteClient(pClient);
        pClient = nullptr;
    }
    if (scaleDevice != nullptr) {
        delete scaleDevice;
        scaleDevice = nullptr;
    }
    collectedMeasurements.clear();
    scaleConnected = false;
    historyRequested = false;
}

void loop() {
    checkRestart();

    if (scaleConnected) {
        if (!historyRequested && (millis() - connectionTime > requestDelayMs)) {
            requestHistoryForAllUsers();
        }

        if (millis() - connectionTime > requestDelayMs + 5000) {
            String jsonString;
            const bool hasMeasurement = buildLatestMeasurementJson(jsonString);
            Serial.println(jsonString);

            Serial.println("Data collection finished. Disconnecting BLE...");
            cleanupBleSession();

            if (hasMeasurement) {
                connectToWifi();
                connectToMqtt();
                if (mqttClient.publish(dataPublishTopic, jsonString.c_str())) {
                    mqttClient.publish(batteryLevelTopic, String(batteryLevel).c_str());
                    Serial.println("Data published to MQTT");
                } else {
                    Serial.println("Failed to publish data");
                }
                mqttClient.loop();
                delay(1000); // wait for publish
                disconnectFromMqtt();
                // disconnectFromWifi();
            } else {
                Serial.println("Skipping MQTT publish: no measurements collected");
            }

            Serial.println("Waiting 40 seconds before restarting...");
            delay(restartDelayMs);
        }
        return;
    }

    if (scaleDevice != nullptr) {
        if (connectToScaleDevice()) {
            Serial.println("Connected to scale device");
        } else {
            Serial.println("Failed to connect, restarting scan...");
            delete scaleDevice;
            scaleDevice = nullptr;
            startScan();
        }
    } else if (!isScanning) {
        startScan();
    }
}

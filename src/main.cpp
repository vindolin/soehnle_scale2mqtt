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

constexpr auto REQUEST_DELAY_MS = 15000; // time to wait before requesting history
constexpr auto COLLECT_DELAY_MS = 5000; // time to wait to collect measurements from notifications
constexpr auto BT_DISCONNECT_DELAY_MS = 40000; // time to wait before the next scan when the last measurement didn't change
// constexpr int SERIAL_STARTUP_DELAY_MS = 4000; // time to wait for Serial to initialize
constexpr auto SERIAL_STARTUP_DELAY_MS = 1000; // time to wait for Serial to initialize

constexpr size_t MQTT_BUFFER_SIZE = 1024;

const auto GMT_OFFSET_SEC = 3600;
const auto DAYLIGHT_OFFSET_SEC = 3600;

const char* BOOT_TIME_TOPIC = "smartscale/bootTime";
const char* BATTERY_LEVEL_TOPIC = "smartscale/battery";

const char* MEASUREMENT_TOPIC = "smartscale/measurement";
const char* MEASUREMENT_TIME_TOPIC = "smartscale/measurementTime";
const char* MEASUREMENT_COUNT_TOPIC = "smartscale/measurementCount";

Measurement latestMeasurement;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

enum class AppState {
    SCANNING,
    CONNECTING,
    CONNECTED_WAIT,
    REQUEST_HISTORY,
    COLLECTING,
    PUBLISHING,
    WAITING
};

AppState currentState = AppState::SCANNING;
unsigned long stateTimer = 0;

NimBLEClient* pClient = nullptr;

const char* SCALE_DEVICE_NAME = "Shape100";

// --- UUIDs ---
static NimBLEUUID SVC_BATTERY("180f");
static NimBLEUUID CHR_BATTERY_LEVEL("2a19");

static NimBLEUUID SVC_CURRENT_TIME("1805");
static NimBLEUUID CHR_CURRENT_TIME("2a2b");

static NimBLEUUID SVC_USER_DATA("181c");
static NimBLEUUID CHR_USER_CONTROL_POINT("2a9f");

static NimBLEUUID SVC_SOEHNLE("352e3000-28e9-40b8-a361-6db4cca4147c");
static NimBLEUUID CHR_SOEHNLE_A("352e3001-28e9-40b8-a361-6db4cca4147c"); // Measurements
// static NimBLEUUID CHR_SOEHNLE_B("352e3004-28e9-40b8-a361-6db4cca4147c"); // ??? always returns 0x01 🤷‍♂️
static NimBLEUUID CHR_SOEHNLE_CMD("352e3002-28e9-40b8-a361-6db4cca4147c"); // Commands

NimBLEAdvertisedDevice* scaleDevice = nullptr;

uint8_t batteryLevel = 0;

Measurement lastPublishedMeasurement;

void setLed(bool on) {
    digitalWrite(BLUE_LED_PIN, on ? LOW : HIGH);
}

uint16_t measurementCount = 0;

bool isBlinking = false;
unsigned long blinkLastToggle = 0;
bool ledState = false;

void updateLed() {
    if (isBlinking) {
        if (millis() - blinkLastToggle > 200) {
            blinkLastToggle = millis();
            ledState = !ledState;
            setLed(ledState);
        }
    }
}

void logHexPayload(const uint8_t* data, size_t length) {
    Serial.print("Received measurement data: ");
    for (size_t i = 0; i < length; i++) {
        Serial.printf("%02x", data[i]);
    }
    Serial.println();
}

// this function get's called for each measurement notification
void notifyMeasurement(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    // logHexPayload(pData, length);

    measurementCount++;

    MeasurementFrame frame;
    if (!parseMeasurementFrame(pData, length, frame)) {
        Serial.println("Skipping invalid measurement frame");
        return;
    }
    if (users.find(frame.pID) == users.end()) {
        Serial.printf("Unknown user pID: %d\n", frame.pID);
        return;
    }
    const User& user = users[frame.pID];

    auto fat = 0.0;
    auto water = 0.0;
    auto muscle = 0.0;
    if(frame.imp50 > 0) {
        fat = calculateFat(user, frame.weightKg, frame.imp50);
        water = calculateWater(user, frame.weightKg, frame.imp50);
        muscle = calculateMuscle(user, frame.weightKg, frame.imp50, frame.imp5);
    }

    char timeStr[25];
    snprintf(timeStr, sizeof(timeStr), "%04d-%02d-%02dT%02d:%02d:%02dZ", frame.year, frame.month, frame.day, frame.hour, frame.minute, frame.second);

    Measurement m;

    m.pID = frame.pID;
    m.time = timeStr;
    m.weight = frame.weightKg;
    m.fat = fat;
    m.water = water;
    m.muscle = muscle;

    Serial.printf("personID %d - %s: weight:%4.1fkg, fat:%4.1f%%, water:%4.1f%%, muscle:%4.1f%%\n",
                  m.pID, m.time.c_str(), m.weight, m.fat, m.water, m.muscle);

    // we keep only the latest measurement from the notifications
    if(latestMeasurement.time.isEmpty() || m.time > latestMeasurement.time) {
        latestMeasurement = m;
    }
}

void notifyBattery(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    if (length > 0) {
        Serial.printf("Battery level: %d%%\n", pData[0]);
    }
}

class ClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pClient) {
        Serial.println("Connected to scale");
    }

    void onDisconnect(NimBLEClient* pClient) {
        Serial.println("Disconnected");
        // We don't delete the client here, we'll handle it in the loop
    }
};

// static ClientCallbacks clientCallbacks;

bool connectToScaleDevice() {
    Serial.printf("Connecting to %s\n", scaleDevice->getAddress().toString().c_str());

    pClient = NimBLEDevice::createClient();
    pClient->setClientCallbacks(new ClientCallbacks());
    // pClient->setClientCallbacks(&clientCallbacks); // this leads to a crash

    if (!pClient->connect(scaleDevice)) {
        Serial.println("Failed to connect");
        NimBLEDevice::deleteClient(pClient);
        pClient = nullptr;
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
            Serial.println("Subscribed to measurements, waiting 15 seconds for history request...");
        }
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

class BLEScanCallbacks: public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice* advertisedDevice) {
        if (advertisedDevice->haveName()) {
            String name = advertisedDevice->getName().c_str();
            String mac = advertisedDevice->getAddress().toString().c_str();
            if (name.indexOf(SCALE_DEVICE_NAME) >= 0) {
                Serial.printf("Found Scale: %s @ %s\n", name.c_str(), mac.c_str());
                NimBLEDevice::getScan()->stop();
                scaleDevice = new NimBLEAdvertisedDevice(*advertisedDevice);
                return;
            }
        }
    }
};

static BLEScanCallbacks scanCallbacks;

void connectToMqtt() {
    uint connectAttempts = 0;
    while (!mqttClient.connected()) {
        if (connectAttempts > 5) {
            ESP.restart();
        }
        connectAttempts++;

        Serial.println("Attempting MQTT connection...");
        if (mqttClient.connect("ESP32ScaleClient", MQTT_SERVER_USER, MQTT_SERVER_PASSWORD)) {
            Serial.println("MQTT connected");
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 2 seconds");
            delay(2000);
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

    uint connectAttempts = 0;
    while (WiFi.status() != WL_CONNECTED) {
        if (connectAttempts > 5) {
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

void disconnectFromWifi() {
    WiFi.disconnect(true);
    Serial.println("WiFi disconnected");
}

void startScan() {
    NimBLEScan* pBLEScan = NimBLEDevice::getScan();
    // pBLEScan->setScanCallbacks(new BLEScanCallbacks());
    pBLEScan->setScanCallbacks(&scanCallbacks);
    if(pBLEScan->start(0, false)) {
        Serial.println("BLE scan started successfully, waiting for scale device...");
        setLed(true);
    } else {
        Serial.println("Failed to start scan");
        setLed(false);
    }
}

void requestHistoryForAllUsers() {
    measurementCount = 0;
    if (pClient == nullptr) return;

    NimBLERemoteService* pSvcSoehnle = pClient->getService(SVC_SOEHNLE);
    if (pSvcSoehnle) {
        NimBLERemoteCharacteristic* pChrSoehnleCmd = pSvcSoehnle->getCharacteristic(CHR_SOEHNLE_CMD);
        if (pChrSoehnleCmd) {
             Serial.println("Requesting history for all users...");
             for (int i = 1; i <= userCount; i++) {
                 uint8_t cmd[] = {MEASUREMENT_OPCODE, (uint8_t)i};
                 pChrSoehnleCmd->writeValue(cmd, 2, true);
                 delay(100);
             }
        }
    }
}

String buildCurrentTimeString() {
    time_t now = time(nullptr);
    struct tm timeinfo;
    if (localtime_r(&now, &timeinfo) != nullptr) {
        char buffer[25];
        strftime(buffer, sizeof(buffer), "%d.%m.%Y - %H:%M:%S ", &timeinfo);
        return String(buffer);
    }
    return String(now);
}

void syncTime() {
    connectToWifi();
    delay(100);

    // Sync time via NTP
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
        connectToMqtt();
        mqttClient.publish(BOOT_TIME_TOPIC, buildCurrentTimeString().c_str());
        mqttClient.loop();
        delay(1000); // wait for publish
        disconnectFromMqtt();

        Serial.println(&timeinfo, "Time synced: %A, %B %d %Y %H:%M:%S");
    } else {
        Serial.println("Failed to sync time");
    }
    disconnectFromWifi();
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

bool generateMeasurementJson(String& outJson) {
    if (latestMeasurement.time.isEmpty()) {
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

    // disable BLE to free memory
    // NimBLEDevice::deinit();
}

void setup() {
    Serial.begin(115200);
    delay(SERIAL_STARTUP_DELAY_MS);  // Wait for CDC Serial to initialize

    pinMode(BLUE_LED_PIN, OUTPUT);
    setLed(false);
    
    mqttClient.setBufferSize(MQTT_BUFFER_SIZE);
    mqttClient.setServer(MQTT_SERVER_IP, MQTT_SERVER_PORT);

    syncTime();
    
    NimBLEDevice::init("ESP32_SCALE");
}

void loop() {
    checkRestart();
    updateLed();

    switch (currentState) {
        case AppState::SCANNING:
            if (scaleDevice != nullptr) {
                currentState = AppState::CONNECTING;
            } else {
                NimBLEScan* pScan = NimBLEDevice::getScan();
                if (!pScan->isScanning()) {
                    startScan();
                }
            }
            break;

        case AppState::CONNECTING:
            setLed(false);
            latestMeasurement = Measurement(); // Clear previous measurement
            if (connectToScaleDevice()) {
                currentState = AppState::CONNECTED_WAIT;
                stateTimer = millis();
            } else {
                Serial.println("Failed to connect, restarting scan...");
                if (scaleDevice != nullptr) {
                    delete scaleDevice;
                    scaleDevice = nullptr;
                }
                currentState = AppState::SCANNING;
            }
            break;

        case AppState::CONNECTED_WAIT:
            if (!pClient || !pClient->isConnected()) {
                 Serial.println("Lost connection during wait!");
                 cleanupBleSession();
                 currentState = AppState::SCANNING;
                 break;
            }
            if (millis() - stateTimer > REQUEST_DELAY_MS) {
                currentState = AppState::REQUEST_HISTORY;
            }
            break;

        case AppState::REQUEST_HISTORY:
            isBlinking = true;
            requestHistoryForAllUsers();
            currentState = AppState::COLLECTING;
            stateTimer = millis();
            break;

        case AppState::COLLECTING:
            if (!pClient || !pClient->isConnected()) {
                 Serial.println("Lost connection during collecting!");
                 cleanupBleSession();
                 isBlinking = false;
                 setLed(false);
                 currentState = AppState::SCANNING;
                 break;
            }
            if (millis() - stateTimer > COLLECT_DELAY_MS) {
                if (latestMeasurement.time == lastPublishedMeasurement.time) {
                    Serial.println("No new measurements received, restarting scan in 40 seconds...");
                    cleanupBleSession();
                    isBlinking = false;
                    setLed(false);
                    currentState = AppState::WAITING;
                    break;
                }
                Serial.println("Measurement collection complete, [" + String(measurementCount) + "]");
                lastPublishedMeasurement = latestMeasurement;
                currentState = AppState::PUBLISHING;
            }
            break;

        case AppState::PUBLISHING:
            {
                String jsonString;
                if (generateMeasurementJson(jsonString)) {
                    Serial.println(jsonString);

                    Serial.println("Data collection finished. Disconnecting BLE...");
                    cleanupBleSession();

                    connectToWifi();
                    connectToMqtt();
                    if (mqttClient.publish(MEASUREMENT_TOPIC, jsonString.c_str())) {
                        Serial.println("Data published to MQTT");
                    } else {
                        Serial.println("Failed to publish data");
                    }
                    mqttClient.publish(BATTERY_LEVEL_TOPIC, String(batteryLevel).c_str());
                    mqttClient.publish(MEASUREMENT_COUNT_TOPIC, String(measurementCount).c_str());

                    const String timeString = buildCurrentTimeString();
                    mqttClient.publish(MEASUREMENT_TIME_TOPIC, timeString.c_str());

                    mqttClient.loop();
                    delay(100); // wait for publish
                    disconnectFromMqtt();
                    // disconnectFromWifi();
                } else {
                    Serial.println("Skipping MQTT publish: no measurements collected");
                    cleanupBleSession();
                }

                Serial.println("Waiting 40 seconds before restarting...");
                isBlinking = false;
                setLed(false);
                currentState = AppState::WAITING;
                stateTimer = millis();
            }
            break;

        case AppState::WAITING:
            if (millis() - stateTimer > BT_DISCONNECT_DELAY_MS) {
                currentState = AppState::SCANNING;
            }
            break;
    }
}

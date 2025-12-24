#include <Arduino.h>
#include <ArduinoJson.h>
#include <NimBLEDevice.h>
#include <NimBLEScan.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <time.h>

#include "config.h"
#include "measurement_helpers.h"

constexpr auto BLUE_LED_PIN = 8;

// LED PWM Configuration
constexpr int PWM_CHANNEL = 0;
constexpr int PWM_FREQ = 5000;
constexpr int PWM_RESOLUTION = 8;
constexpr int MAX_DUTY = 255;
constexpr int DEFAULT_MAX_BRIGHTNESS = (MAX_DUTY * 0.3); // 30% brightness
int currentMaxBrightness = DEFAULT_MAX_BRIGHTNESS;

constexpr auto REQUEST_DELAY_MS = 15000;       // time to wait before requesting history
constexpr auto BT_DISCONNECT_DELAY_MS = 45000; // time to wait before the next scan when the last measurement
                                               // didn't change
// constexpr int SERIAL_STARTUP_DELAY_MS = 4000; // time to wait for Serial to
// initialize
constexpr auto SERIAL_STARTUP_DELAY_MS = 1000; // time to wait for Serial to initialize

constexpr size_t MQTT_BUFFER_SIZE = 1024;

const auto GMT_OFFSET_SEC = 3600;
const auto DAYLIGHT_OFFSET_SEC = 3600;

const char *MAIN_TOPIC = "smartscale2/";

const char *BOOT_TIME_TOPIC = "bootTime";
const char *BATTERY_LEVEL_TOPIC = "battery";

const char *MEASUREMENT_TOPIC = "measurement";
const char *MEASUREMENT_TIME_TOPIC = "measurementTime";
const char *MEASUREMENT_COUNT_TOPIC = "measurementCount";

const char *LOOP_COUNT_TOPIC = "loopCount";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

enum class AppState { SCANNING, CONNECTING, CONNECTED_WAIT, WAIT_FOR_MEASUREMENT, PUBLISH_MEASUREMENT, WAIT_FOR_SCALE_TO_DISAPPEAR };

AppState currentAppState = AppState::SCANNING;
unsigned long stateTimer = 0;

enum class BlinkState { OFF, ON };

enum class LedMode {
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

static NimBLEUUID SVC_BODY_COMPOSITION("0000181b-0000-1000-8000-00805f9b34fb");
static NimBLEUUID CHR_BODY_COMPOSITION_MEASUREMENT("00002a9c-0000-1000-8000-00805f9b34fb");

NimBLEAdvertisedDevice *scaleDevice = nullptr;

uint8_t batteryLevel = 0;

Measurement lastPublishedMeasurement;

void setLed(bool on) {
    // Active LOW: 0 is ON (Max brightness), 255 is OFF.
    int duty = on ? (MAX_DUTY - currentMaxBrightness) : MAX_DUTY;
    ledcWrite(PWM_CHANNEL, duty);
}

bool ledState = false;

void toggleLed() {
    // static bool ledState = false;
    ledState = !ledState;
    setLed(ledState);
}

uint16_t measurementCount = 0;
uint32_t loopCount = 0;

void logHexPayload(const uint8_t *data, size_t length) {
    Serial.print("Received measurement data: ");
    for (size_t i = 0; i < length; i++) {
        Serial.printf("%02x", data[i]);
    }
    Serial.println();
}

void processMeasurementPayload(const uint8_t *data, size_t length) {
    measurementCount++;

    Measurement measurement;

    if (buildMeasurementFromBodyCompositionFrame(data, length, measurement)) {
        storeMeasurement(measurement);
        currentAppState = AppState::PUBLISH_MEASUREMENT;
        return;
    }

    Serial.println("Skipping measurement payload: unsupported format");
}

void indicateBodyComposition(NimBLERemoteCharacteristic *pRemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
    Serial.println("Body Composition indication received:");
    logHexPayload(pData, length);
    processMeasurementPayload(pData, length);
}

bool connectToScaleDevice() {
    Serial.printf("Connecting to %s\n", scaleDevice->getAddress().toString().c_str());

    pClient = NimBLEDevice::createClient();

    if (!pClient->connect(scaleDevice)) {
        Serial.println("Failed to connect");
        NimBLEDevice::deleteClient(pClient);
        pClient = nullptr;
        return false;
    }

    // Battery Service
    NimBLERemoteService *pSvcBattery = pClient->getService(SVC_BATTERY);
    if (pSvcBattery) {
        NimBLERemoteCharacteristic *pChrBattery = pSvcBattery->getCharacteristic(CHR_BATTERY_LEVEL);
        if (pChrBattery) {
            if (pChrBattery->canRead()) {
                std::string value = pChrBattery->readValue();
                if (value.length() > 0) {
                    batteryLevel = value[0];
                    Serial.printf("Initial Battery: %d%%\n", batteryLevel);
                }
            }
        }
    }

    // Current Time Service
    NimBLERemoteService *pSvcTime = pClient->getService(SVC_CURRENT_TIME);
    if (pSvcTime) {
        NimBLERemoteCharacteristic *pChrTime = pSvcTime->getCharacteristic(CHR_CURRENT_TIME);
        Serial.println(pChrTime->getValue()); // just to check if it exists

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

    NimBLERemoteService *pSvcBodyComposition = pClient->getService(SVC_BODY_COMPOSITION);
    if (pSvcBodyComposition) {
        Serial.println("Found Body Composition Service");
        NimBLERemoteCharacteristic *pChrBodyComposition = pSvcBodyComposition->getCharacteristic(CHR_BODY_COMPOSITION_MEASUREMENT);
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

bool disconnectFromScaleDevice() {
    if (pClient && pClient->isConnected()) {
        pClient->disconnect();
        return true;
    }
    return false;
}

class BLEScanCallbacks : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice *advertisedDevice) {
        if (advertisedDevice->haveName()) {
            String name = advertisedDevice->getName().c_str();
            String mac = advertisedDevice->getAddress().toString().c_str();

            // Serial.printf("Device: %s @ %s\n", name.c_str(), mac.c_str());

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
        if (mqttClient.connect("ESP32ScaleClientX", MQTT_SERVER_USER, MQTT_SERVER_PASSWORD)) {
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
    NimBLEScan *pBLEScan = NimBLEDevice::getScan();
    pBLEScan->setScanCallbacks(&scanCallbacks);
    if (pBLEScan->start(0, false)) {
        Serial.println("BLE scan started successfully, waiting for scale device...");
    } else {
        Serial.println("Failed to start scan");
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
        mqttClient.publish((String(MAIN_TOPIC) + BOOT_TIME_TOPIC).c_str(), buildCurrentTimeString().c_str(), true);
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

bool generateMeasurementJson(String &outJson) {
    if (latestMeasurement.time.isEmpty()) {
        return false;
    }
    DynamicJsonDocument doc(256);

    doc["p_id"] = latestMeasurement.pID;
    doc["time"] = latestMeasurement.time;
    doc["weight"] = round(latestMeasurement.weight * 10.0) / 10.0;
    doc["fat"] = round(latestMeasurement.fat * 10.0) / 10.0;
    doc["water"] = round(latestMeasurement.water * 10.0) / 10.0;
    doc["muscle"] = round(latestMeasurement.muscle * 10.0) / 10.0;
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

void setLedBlinkDurations(uint16_t onDurationMs, uint16_t offDurationMs) {
    blinkOnDurationMs = onDurationMs;
    blinkOffDurationMs = offDurationMs;
}

void setLedModeBlink(uint16_t onDurationMs = 200, uint16_t offDurationMs = 800) {
    currentLedMode = LedMode::PULSE;
    currentMaxBrightness = DEFAULT_MAX_BRIGHTNESS;
    setLedBlinkDurations(onDurationMs, offDurationMs);
}

void setLedModeOff() {
    currentLedMode = LedMode::OFF;
    setLed(false);
}

void loop() {
    checkRestart();

    switch (currentAppState) {
        case AppState::SCANNING:
            if (scaleDevice != nullptr) {
                currentAppState = AppState::CONNECTING;
            } else {
                NimBLEScan *pScan = NimBLEDevice::getScan();
                if (!pScan->isScanning()) {
                    setLedModeBlink(100, 2000);
                    startScan();
                }
            }
            break;

        case AppState::CONNECTING:
            loopCount++;

            setLedModeBlink(50, 100);

            latestMeasurement = Measurement(); // Clear previous measurement
            if (connectToScaleDevice()) {
                setLedModeBlink(100, 100);
                currentAppState = AppState::CONNECTED_WAIT;
                stateTimer = millis();
            } else {
                Serial.println("Failed to connect, restarting scan...");
                if (scaleDevice != nullptr) {
                    delete scaleDevice;
                    scaleDevice = nullptr;
                }
                currentAppState = AppState::SCANNING;
            }
            break;

        case AppState::CONNECTED_WAIT:
            if (!pClient || !pClient->isConnected()) {
                Serial.println("Lost connection during wait!");
                cleanupBleSession();
                currentAppState = AppState::SCANNING;
                break;
            }
            if (millis() - stateTimer > REQUEST_DELAY_MS) {
                currentAppState = AppState::WAIT_FOR_MEASUREMENT;
            }
            break;

        case AppState::WAIT_FOR_MEASUREMENT:
            if (!pClient || !pClient->isConnected()) {
                Serial.println("Lost connection while waiting for measurement!");
                cleanupBleSession();
                currentAppState = AppState::SCANNING;
                break;
            }
            break;

        case AppState::PUBLISH_MEASUREMENT:
            disconnectFromScaleDevice();

            connectToWifi();
            connectToMqtt();

            // Publish battery level
            char batteryStr[4];
            snprintf(batteryStr, sizeof(batteryStr), "%d", batteryLevel);
            mqttClient.publish((String(MAIN_TOPIC) + BATTERY_LEVEL_TOPIC).c_str(), batteryStr, true);

            // Publish measurement
            {
                String measurementJson;
                if (generateMeasurementJson(measurementJson)) {
                    mqttClient.publish((String(MAIN_TOPIC) + MEASUREMENT_TOPIC).c_str(), measurementJson.c_str(), true);
                    mqttClient.publish((String(MAIN_TOPIC) + MEASUREMENT_TIME_TOPIC).c_str(), latestMeasurement.time.c_str(), true);

                    char countStr[12];
                    snprintf(countStr, sizeof(countStr), "%d", measurementCount);
                    mqttClient.publish((String(MAIN_TOPIC) + MEASUREMENT_COUNT_TOPIC).c_str(), countStr, true);

                    Serial.printf("Published measurement: %s\n", measurementJson.c_str());
                } else {
                    Serial.println("No valid measurement to publish");
                }
            }

            mqttClient.loop();
            delay(1000); // wait for publishes

            disconnectFromMqtt();

            lastPublishedMeasurement = latestMeasurement;

            stateTimer = millis();
            currentAppState = AppState::WAIT_FOR_SCALE_TO_DISAPPEAR;
            break;

        case AppState::WAIT_FOR_SCALE_TO_DISAPPEAR:
            currentMaxBrightness = map(millis() - stateTimer, 0, BT_DISCONNECT_DELAY_MS, DEFAULT_MAX_BRIGHTNESS, 0);
            currentMaxBrightness = constrain(currentMaxBrightness, 0, DEFAULT_MAX_BRIGHTNESS);

            if (millis() - stateTimer > BT_DISCONNECT_DELAY_MS) {
                currentAppState = AppState::SCANNING;
            }
            break;

    } // end switch app state

    switch (currentLedMode) {
        case LedMode::PULSE: {
            unsigned long period = blinkOnDurationMs + blinkOffDurationMs;
            if (period == 0)
                period = 1000;

            float phase = (float)(millis() % period) / period * 2 * PI;
            float val = (1.0f - cos(phase)) / 2.0f;

            int brightness = val * currentMaxBrightness;
            int duty = MAX_DUTY - brightness; // Active LOW
            ledcWrite(PWM_CHANNEL, duty);
            break;
        }

        case LedMode::OFF:
            break;
    } // end switch led mode
}

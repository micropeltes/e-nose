#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Adafruit_ADS1X15.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include "mqtt_ca_cert.h"

// I2C
constexpr uint8_t I2C_SDA_PIN = 10;
constexpr uint8_t I2C_SCL_PIN = 6;
Adafruit_ADS1115 ads1;

// ADC values
int16_t adc1_0, adc1_1, adc1_2, adc1_3;
bool ads1Ok = false;

// WIFI + MQTT
const char* ssid = "POCO X7 Pro";
const char* password = "1234567i";
const char* mqtt_server = "45.126.43.35";
const uint16_t mqtt_port = 8883;
const char* mqtt_pub_topic = "test/topic";
const char* mqtt_err_topic = "sensor/errorlog";
const char* mqtt_interval_sub_topic = "sensor/control";
constexpr uint32_t MQTT_INTERVAL_MIN_MS = 200;
constexpr uint32_t MQTT_INTERVAL_MAX_MS = 60000;
volatile uint32_t mqttPublishIntervalMs = 1000;

WiFiClientSecure espClient;
PubSubClient client(espClient);

// TASK HANDLE
TaskHandle_t TaskSensorHandle;
TaskHandle_t TaskMQTTHandle;

void publishErrorLog(const char* msg) {
  if (!client.connected()) return;
  client.publish(mqtt_err_topic, msg);
}

bool parseIntervalFromPayload(const byte* payload, unsigned int length, uint32_t& outMs) {
  char msg[64];
  unsigned int copyLen = (length < sizeof(msg) - 1) ? length : (sizeof(msg) - 1);
  memcpy(msg, payload, copyLen);
  msg[copyLen] = '\0';

  char* p = msg;
  while (*p != '\0' && (*p < '0' || *p > '9')) {
    p++;
  }
  if (*p == '\0') {
    return false;
  }

  char* endPtr = nullptr;
  unsigned long value = strtoul(p, &endPtr, 10);
  if (endPtr == p) {
    return false;
  }

  if (value < MQTT_INTERVAL_MIN_MS || value > MQTT_INTERVAL_MAX_MS) {
    return false;
  }

  outMs = static_cast<uint32_t>(value);
  return true;
}

void handleIntervalConfigMessage(const byte* payload, unsigned int length) {
  uint32_t newIntervalMs = 0;
  if (parseIntervalFromPayload(payload, length, newIntervalMs)) {
    mqttPublishIntervalMs = newIntervalMs;
    Serial.print("MQTT interval updated to ");
    Serial.print(mqttPublishIntervalMs);
    Serial.println(" ms");
  } else {
    Serial.print("Invalid interval payload. Use ");
    Serial.print(MQTT_INTERVAL_MIN_MS);
    Serial.print("..");
    Serial.print(MQTT_INTERVAL_MAX_MS);
    Serial.println(" (ms)");
    publishErrorLog("Invalid interval payload");
  }
}

// CALLBACK MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message on: ");
  Serial.println(topic);

  if (strcmp(topic, mqtt_interval_sub_topic) == 0) {
    handleIntervalConfigMessage(payload, length);
  }
}

// ================= WIFI =================
void connectWiFi() {
  WiFi.begin(ssid, password);

  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nWiFi Connected");
}

// ================= I2C SCAN =================
void scanI2C() {
  Serial.println("Scanning I2C...");
  for (byte i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(i, HEX);
    }
  }
}

// ================= MQTT RECONNECT (NON BLOCKING) =================
void reconnectMQTT() {

  if (client.connected()) return;

  Serial.println("MQTT reconnecting...");

  if (client.connect("ESP32C3_CLIENT")) {
    Serial.println("MQTT Connected");
    client.subscribe(mqtt_interval_sub_topic);
  } else {
    Serial.print("MQTT Failed, rc=");
    Serial.println(client.state());
  }
}

// ================= TASK SENSOR =================
void TaskSensor(void * parameter) {

  for (;;) {

    if (ads1Ok) {
      adc1_0 = ads1.readADC_SingleEnded(0);
      adc1_1 = ads1.readADC_SingleEnded(1);
      adc1_2 = ads1.readADC_SingleEnded(2);
      adc1_3 = ads1.readADC_SingleEnded(3);
    } else {
      adc1_0 = -1;
      adc1_1 = -1;
      adc1_2 = -1;
      adc1_3 = -1;
    }

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// ================= TASK MQTT =================
void TaskMQTT(void * parameter) {

  char payload[256];

  for (;;) {

    reconnectMQTT();
    client.loop();

    snprintf(payload, sizeof(payload),
      "{\"devid\":\"esp32-002\",\"nh3_mics\":8500,\"nh3_mems\":8700,\"h2s\":9100,\"no2\":7600,\"co\":10200,\"mq135\":9500}");

    if (client.connected()) {

      bool ok = client.publish(mqtt_pub_topic, payload);

      if (!ok) {
        Serial.println("Publish gagal -> sensor/errorlog");
        publishErrorLog("Publish sensor payload gagal");
        client.publish(mqtt_err_topic, payload);
      }

    } else {
      Serial.println("MQTT tidak connected");
    }

    Serial.println(payload);

    vTaskDelay(mqttPublishIntervalMs / portTICK_PERIOD_MS);
  }
}

// ================= SETUP =================
void setup() {

  Serial.begin(115200);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  scanI2C();

  ads1Ok = ads1.begin(0x48);
  if (!ads1Ok) {
    Serial.println("ADS1115 NOT DETECTED");
  }

  connectWiFi();

  // ================= SYNC WAKTU (WAJIB UNTUK TLS) =================
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  Serial.print("Sync time");
  time_t now = time(nullptr);
  while (now < 100000) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println(" OK");

  // ================= TLS CONFIG =================
  espClient.setInsecure();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // ================= TASK =================
  xTaskCreate(TaskSensor, "SensorTask", 4096, NULL, 1, &TaskSensorHandle);
  xTaskCreate(TaskMQTT, "MQTTTask", 4096, NULL, 1, &TaskMQTTHandle);
}

void loop() {
}

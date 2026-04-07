#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Adafruit_ADS1X15.h>
#include <LiquidCrystal_I2C.h>
#include "mqtt_ca_cert.h"

// I2C
Adafruit_ADS1115 ads1;
Adafruit_ADS1115 ads2;

LiquidCrystal_I2C lcd1(0x27, 16, 2);
LiquidCrystal_I2C lcd2(0x20, 16, 2);

// ADC values
int16_t adc1_0, adc1_1, adc1_2, adc1_3;
int16_t adc2_0, adc2_1, adc2_2, adc2_3;
bool ads1Ok = false;
bool ads2Ok = false;

// WIFI + MQTT
const char* ssid = "POCO X7 Pro";
const char* password = "1234567i";
const char* mqtt_server = "45.126.43.35";
const uint16_t mqtt_port = 8883;

WiFiClientSecure espClient;
PubSubClient client(espClient);

TaskHandle_t TaskSensorHandle;
TaskHandle_t TaskMQTTHandle;
TaskHandle_t TaskLCDHandle;

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Topic: ");
  Serial.println(topic);
}

// WIFI CONNECT
void connectWiFi() {

  WiFi.begin(ssid, password);

  Serial.print("Connecting WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("\nWiFi Connected");
}

// MQTT RECONNECT
void reconnectMQTT() {

  while (!client.connected()) {

    Serial.println("MQTT reconnecting...");

    if (client.connect("ESP32C3_CLIENT")) {

      client.subscribe("sensor/control");

      Serial.println("MQTT Connected");

    } else {

      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
  }
}

// SENSOR TASK
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

    if (ads2Ok) {
      adc2_0 = ads2.readADC_SingleEnded(0);
      adc2_1 = ads2.readADC_SingleEnded(1);
      adc2_2 = ads2.readADC_SingleEnded(2);
      adc2_3 = ads2.readADC_SingleEnded(3);
    } else {
      adc2_0 = -1;
      adc2_1 = -1;
      adc2_2 = -1;
      adc2_3 = -1;
    }

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// MQTT TASK
void TaskMQTT(void * parameter) {

  for (;;) {

    if (!client.connected()) {
      reconnectMQTT();
    }

    client.loop();

    String payload = "{"
    "\"devid\":\"ESP-00\""+
    ",\"nh3_mics\":"+String(adc1_1)+
    ",\"nh3_mems\":"+String(adc2_3)+
    ",\"h2s\":"+String(adc2_2)+
    ",\"no2\":"+String(adc1_0)+
    ",\"co\":"+String(adc1_2)+
    ",\"mq135\":"+String(adc1_3)+
    ",\"ads1_status\":\""+String(ads1Ok ? "ok" : "error")+"\""+
    ",\"ads2_status\":\""+String(ads2Ok ? "ok" : "error")+"\""+
    "}";

    client.publish("sensor/gas", payload.c_str());

    Serial.println(payload);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// LCD TASK
void TaskLCD(void * parameter) {

  for (;;) {

    lcd1.setCursor(0,0);
    lcd1.print("A1:");
    lcd1.print(adc1_0);
    
    lcd1.setCursor(0,1);
    lcd1.print("A2:");
    lcd1.print(adc1_1);
    
    lcd1.setCursor(8,0);
    lcd1.print("A3:");
    lcd1.print(adc1_2);
      
    lcd1.setCursor(8,1);
    lcd1.print("A4:");
    lcd1.print(adc1_3);
    
    lcd2.setCursor(0,0);
    lcd2.print("A5:");
    lcd2.print(adc2_0);
    
    lcd2.setCursor(0,1);
    lcd2.print("A6:");
    lcd2.print(adc2_1);
    
    lcd2.setCursor(8,0);
    lcd2.print("A7:");
    lcd2.print(adc2_2);
    
    lcd2.setCursor(8,1);
    lcd2.print("A8:");
    lcd2.print(adc2_3);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// SETUP
void setup() {

  Serial.begin(115200);

  Wire.begin();

  lcd1.init();
  lcd1.backlight();

  lcd2.init();
  lcd2.backlight();

  ads1Ok = ads1.begin(0x48);
  if (!ads1Ok) {
    lcd1.print("ADS1 ERROR");
  }

  ads2Ok = ads2.begin(0x4A);
  if (!ads2Ok) {
    lcd1.setCursor(0,1);
    lcd1.print("ADS2 ERROR");
  }

  connectWiFi();

  espClient.setCACert(MQTT_CA_CERT);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // CREATE TASKS
  xTaskCreate(
    TaskSensor,
    "SensorTask",
    4096,
    NULL,
    1,
    &TaskSensorHandle
  );

  xTaskCreate(
    TaskMQTT,
    "MQTTTask",
    4096,
    NULL,
    1,
    &TaskMQTTHandle
  );

  xTaskCreate(
    TaskLCD,
    "LCDTask",
    2048,
    NULL,
    1,
    &TaskLCDHandle
  );
}

void loop() {
}

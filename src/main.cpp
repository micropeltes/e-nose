#include <Arduino.h>
#include <Wire.h>
<<<<<<< Updated upstream
#include <WiFi.h>
#include <PubSubClient.h>
=======
>>>>>>> Stashed changes
#include <Adafruit_ADS1X15.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "mqtt_ca_cert.h"

<<<<<<< Updated upstream
// I2C
Adafruit_ADS1115 ads1;
Adafruit_ADS1115 ads2;
=======
// I2C defines
Adafruit_ADS1115 ads1;
Adafruit_ADS1115 ads2;
LiquidCrystal_I2C lcd1(0x27, 16, 2);
LiquidCrystal_I2C lcd2(0x20, 16, 2);

// ADS1
int16_t adc1_0 = ads1.readADC_SingleEnded(0);
int16_t adc1_1 = ads1.readADC_SingleEnded(1);
int16_t adc1_2 = ads1.readADC_SingleEnded(2);
int16_t adc1_3 = ads1.readADC_SingleEnded(3);
// ADS2
int16_t adc2_0 = ads2.readADC_SingleEnded(0);
int16_t adc2_1 = ads2.readADC_SingleEnded(1);
int16_t adc2_2 = ads2.readADC_SingleEnded(2);
int16_t adc2_3 = ads2.readADC_SingleEnded(3);
>>>>>>> Stashed changes

LiquidCrystal_I2C lcd1(0x27, 16, 2);
LiquidCrystal_I2C lcd2(0x20, 16, 2);

// ADC values
int16_t adc1_0, adc1_1, adc1_2, adc1_3;
int16_t adc2_0, adc2_1, adc2_2, adc2_3;

// WIFI + MQTT
const char* ssid = "POCO X7 Pro";
const char* password = "1234567i";
const char* mqtt_server = "45.126.43.35";
const uint16_t mqtt_port = 1199;

WiFiClientSecure espClient;
PubSubClient client(espClient);
TaskHandle_t TaskMQTTHandle = NULL;

<<<<<<< Updated upstream
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
=======
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.println("MQTT reconnecting...");
    if (client.connect("ESP32_CLIENT_1")) {
      client.subscribe("sensor/control");
      Serial.println("MQTT Connected!");
    }
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream

  for (;;) {

    adc1_0 = ads1.readADC_SingleEnded(0);
    adc1_1 = ads1.readADC_SingleEnded(1);
    adc1_2 = ads1.readADC_SingleEnded(2);
    adc1_3 = ads1.readADC_SingleEnded(3);

    adc2_0 = ads2.readADC_SingleEnded(0);
    adc2_1 = ads2.readADC_SingleEnded(1);
    adc2_2 = ads2.readADC_SingleEnded(2);
    adc2_3 = ads2.readADC_SingleEnded(3);

=======
  
  for (;;) {
>>>>>>> Stashed changes
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

<<<<<<< Updated upstream
// MQTT TASK
=======
>>>>>>> Stashed changes
void TaskMQTT(void * parameter) {

  for (;;) {

    if (!client.connected()) {
      reconnectMQTT();
    }

    client.loop();

<<<<<<< Updated upstream
    String payload = "{"
    "\"adc1\":"+String(adc1_0)+
    ",\"adc2\":"+String(adc1_1)+
    ",\"adc3\":"+String(adc1_2)+
    ",\"adc4\":"+String(adc1_3)+
    ",\"adc5\":"+String(adc2_0)+
    ",\"adc6\":"+String(adc2_1)+
    ",\"adc7\":"+String(adc2_2)+
    ",\"adc8\":"+String(adc2_3)+
    "}";
=======
    String payload = "{\"MQ135_ADC\"}";
>>>>>>> Stashed changes

    client.publish("sensor/gas", payload.c_str());

    Serial.println(payload);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

<<<<<<< Updated upstream
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

  if (!ads1.begin(0x48)) {
    lcd1.print("ADS1 ERROR");
    while(1);
  }

  if (!ads2.begin(0x4A)) {
    lcd1.setCursor(0,1);
    lcd1.print("ADS2 ERROR");
    while(1);
  }

  connectWiFi();
=======
void setup() {
  Serial.begin(115200);
  Wire.begin();
  espClient.setCACert(MQTT_CA_CERT);
  client.setServer(mqtt_server, mqtt_port);

  lcd1.init();
  lcd1.backlight();

  // ADS1 ADDR -> GND mq135 dan mems h2s
  lcd1.setCursor(0,0);
  if (!ads1.begin(0x48)) {
    lcd1.setCursor(0,0);
    lcd1.print("ADS1 gagal");
    while (1);
  }
  else{
>>>>>>> Stashed changes

  }

  // ADS2 ADDR -> SDA mems nh3 dan mics 6814
  if (!ads2.begin(0x4A)) {
    lcd1.setCursor(0,1);
    lcd1.print("ADS2 gagal");
    while (1);
  }

  lcd1.setCursor(0,0);
  lcd1.print("ADS1115 Ready");
  delay(2000);
  lcd1.clear();

<<<<<<< Updated upstream
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
=======
  xTaskCreatePinnedToCore(TaskMQTT, "TaskMQTT", 4096, NULL, 1, &TaskMQTTHandle, 1);
}

void loop() {
  Serial.print("ADS1 CH0: "); Serial.print(adc1_0);
  Serial.print(" | CH1: "); Serial.print(adc1_0);

  Serial.print(" || ADS2 CH0: "); Serial.print(adc1_0);
  Serial.print(" | CH1: "); Serial.println(adc1_0);

  delay(1000);
>>>>>>> Stashed changes
}

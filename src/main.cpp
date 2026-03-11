#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_ADS1X15.h>
#include <LiquidCrystal_I2C.h>

// I2C
Adafruit_ADS1115 ads1;
Adafruit_ADS1115 ads2;

LiquidCrystal_I2C lcd1(0x27, 16, 2);
LiquidCrystal_I2C lcd2(0x20, 16, 2);

// ADC values
int16_t adc1_0, adc1_1, adc1_2, adc1_3;
int16_t adc2_0, adc2_1, adc2_2, adc2_3;

// WIFI + MQTT
const char* ssid = "POCO X7 Pro";
const char* password = "1234567i";
const char* mqtt_server = "broker.emqx.io";

WiFiClient espClient;
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

    adc1_0 = ads1.readADC_SingleEnded(0);
    adc1_1 = ads1.readADC_SingleEnded(1);
    adc1_2 = ads1.readADC_SingleEnded(2);
    adc1_3 = ads1.readADC_SingleEnded(3);

    adc2_0 = ads2.readADC_SingleEnded(0);
    adc2_1 = ads2.readADC_SingleEnded(1);
    adc2_2 = ads2.readADC_SingleEnded(2);
    adc2_3 = ads2.readADC_SingleEnded(3);

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
    "\"adc1\":"+String(adc1_0)+
    ",\"adc2\":"+String(adc1_1)+
    ",\"adc3\":"+String(adc1_2)+
    ",\"adc4\":"+String(adc1_3)+
    ",\"adc5\":"+String(adc2_0)+
    ",\"adc6\":"+String(adc2_1)+
    ",\"adc7\":"+String(adc2_2)+
    ",\"adc8\":"+String(adc2_3)+
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

  client.setServer(mqtt_server, 1883);
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

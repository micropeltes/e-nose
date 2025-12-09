#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2);

const char* ssid = "POCO X7 Pro";
const char* password = "1234567i";
const char* mqtt_server = "broker.emqx.io";

WiFiClient espClient;
PubSubClient client(espClient);

// Sensor Values
int adc_mq135 = 0;
int adc_mems  = 0;

float Vadc_mq135, Vsensor_mq135, Rs_mq135;
float Vadc_mems;

// FreeRTOS Tasks
TaskHandle_t TaskSensorHandle;
TaskHandle_t TaskLCDHandle;
TaskHandle_t TaskMQTTHandle;


// MQTT Callback
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Topic: ");
  Serial.println(topic);
}


// Reconnect MQTT
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.println("MQTT reconnecting...");
    if (client.connect("ESP32_CLIENT_1")) {
      client.subscribe("sensor/control");
      Serial.println("MQTT Connected!");
    }
    delay(500);
  }
}


// Task Sensor (Core 0)
void TaskSensor(void * parameter) {

  const float VCC = 3.3;
  const float RLOAD = 3000.0;  // 1k + 2k (ohm)

  for (;;) {

    // Baca ADC
    adc_mq135 = analogRead(34);
    adc_mems  = analogRead(35);

    // MQ135 — hitung Rs
    float Vnode_mq135 = adc_mq135 * (VCC / 4095.0);
    Rs_mq135 = (RLOAD * ((VCC / Vnode_mq135) - 1.0)); // ohm
    float Rs_kohm = Rs_mq135 / 1000.0; // tampilkan kΩ

    // MEMS Sensor
    Vadc_mems = adc_mems * (VCC / 4095.0);

    // Debug Serial
    Serial.printf(
      "MQ135 ADC:%d  V:%.3fV  Rs:%.2f kΩ  | MEMS ADC:%d  V:%.2f\n",
      adc_mq135, Vnode_mq135, Rs_kohm, adc_mems, Vadc_mems
    );

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// Task LCD (Core 1)
void TaskLCD(void * parameter) {
  lcd.init();
  lcd.backlight();

  for (;;) {
    lcd.setCursor(0,0); 
    lcd.print("MQ Rs: ");
    lcd.print(Rs_mq135, 1);
    lcd.print("k ");

    lcd.setCursor(0,1);
    lcd.print("MEMS: ");
    lcd.print(Vadc_mems, 2);
    lcd.print("V  ");

    vTaskDelay(800 / portTICK_PERIOD_MS);
  }
}


// Task MQTT (Core 1)
void TaskMQTT(void * parameter) {
  for (;;) {
    if (!client.connected()) {
      reconnectMQTT();
    }

    client.loop();

    String payload = "{\"MQ135_ADC\":"+String(adc_mq135)+
                     ",\"MQ135_Rs\":"+String(Rs_mq135,2)+
                     ",\"MEMS_ADC\":"+String(adc_mems)+
                     ",\"MEMS_V\":"+String(Vadc_mems,2)+"}";

    client.publish("sensor/gas", payload.c_str());

    Serial.println("MQTT Payload: " + payload);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}


// SETUP
void setup() {
  Serial.begin(9600);

  pinMode(34, INPUT);
  pinMode(35, INPUT);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println("\nWiFi connected!");

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  xTaskCreatePinnedToCore(TaskSensor, "TaskSensor", 4096, NULL, 1, &TaskSensorHandle, 0);
  xTaskCreatePinnedToCore(TaskLCD, "TaskLCD", 4096, NULL, 1, &TaskLCDHandle, 1);
  xTaskCreatePinnedToCore(TaskMQTT, "TaskMQTT", 4096, NULL, 1, &TaskMQTTHandle, 1);
}


void loop() {}

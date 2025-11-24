#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);

// ------------------ WiFi & MQTT ------------------
const char* ssid = "WIFI_KAMU";
const char* password = "PASSWORD_KAMU";
const char* mqtt_server = "broker.emqx.io";

WiFiClient espClient;
PubSubClient client(espClient);

// ------------------ Sensor Values ------------------
int mq2 = 0;
int mq7 = 0;
int mq135 = 0;

// ------------------ RTOS Handles ------------------
TaskHandle_t TaskSensorCore0;
TaskHandle_t TaskLCD_MqttCore1;


// ===============================
// Fungsi untuk mengirim MQTT
// ===============================
void sendMQTTData() {
  if (!client.connected()) {
    Serial.println("MQTT reconnect...");
    while (!client.connected()) {
      client.connect("ESP32_CLIENT_1");
      delay(500);
    }
  }

  String payload = "{\"MQ2\":" + String(mq2) +
                   ",\"MQ7\":" + String(mq7) +
                   ",\"MQ135\":" + String(mq135) + "}";

  client.publish("sensor/gas", payload.c_str());
  Serial.println("MQTT Sent: " + payload);
}


// ===============================
// Task Core 0: Pembacaan Sensor
// ===============================
void TaskSensor(void * parameter) {
  for (;;) {
    mq2 = analogRead(34);
    mq7 = analogRead(35);
    mq135 = analogRead(25);

    Serial.printf("MQ2: %d | MQ7: %d | MQ135: %d\n", mq2, mq7, mq135);

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}


// ===============================
// Task Core 1: LCD + MQTT
// ===============================
void TaskLCD_MQTT(void * parameter) {
  lcd.init();
  lcd.backlight();

  for (;;) {
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("MQ2 : ");  lcd.print(mq2);
    lcd.setCursor(0,1); lcd.print("MQ7 : ");  lcd.print(mq7);
    lcd.setCursor(0,2); lcd.print("MQ135: "); lcd.print(mq135);

    sendMQTTData();   // 🟢 KIRIM MQTT

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}


// ===============================
// Setup
// ===============================
void setup() {
  Serial.begin(115200);

  pinMode(34, INPUT);
  pinMode(35, INPUT);
  pinMode(25, INPUT);

  // WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println("\nConnected!");
  
  client.setServer(mqtt_server, 1883);

  // Buat Task ● Core 0 → Sensor
  xTaskCreatePinnedToCore(
    TaskSensor, "TaskSensor", 4096, NULL, 1, &TaskSensorCore0, 0);

  // Buat Task ● Core 1 → LCD + MQTT
  xTaskCreatePinnedToCore(
    TaskLCD_MQTT, "TaskLCD_MQTT", 4096, NULL, 1, &TaskLCD_MqttCore1, 1);
}

void loop() {
  // HAL kosong, RTOS yang bekerja
}

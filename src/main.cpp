#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);

// -------------------- WiFi & MQTT --------------------
const char* ssid = "Cove_SIMS";
const char* password = "lovewhereyoulive";
const char* mqtt_server = "broker.emqx.io";

WiFiClient espClient;
PubSubClient client(espClient);

// -------------------- Sensor Values --------------------
int mq2 = 0;
int mq7 = 0;
int mq135 = 0;

// -------------------- RTOS Handle --------------------
TaskHandle_t TaskSensorHandle;
TaskHandle_t TaskLCDHandle;
TaskHandle_t TaskMQTTHandle;


// -------------------------------------------------------
// MQTT Callback
// -------------------------------------------------------
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("MQTT Msg Topic: ");
  Serial.println(topic);

  Serial.print("Payload: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}


// -------------------------------------------------------
// MQTT Reconnect
// -------------------------------------------------------
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.println("MQTT reconnecting...");
    if (client.connect("ESP32_CLIENT_1")) {
      client.subscribe("sensor/control", 0);  // QoS 0
      Serial.println("MQTT Connected!");
    }
    delay(500);
  }
}


// -------------------------------------------------------
// Task 1 → SENSOR (Core 0)
// -------------------------------------------------------
void TaskSensor(void * parameter) {
  for (;;) {
    mq2   = analogRead(34);
    mq7   = analogRead(35);
    mq135 = analogRead(36);

    Serial.printf("MQ2:%d  MQ7:%d  MQ135:%d\n", mq2, mq7, mq135);

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}


// -------------------------------------------------------
// Task 2 → LCD (Core 1)
// -------------------------------------------------------
void TaskLCD(void * parameter) {
  lcd.init();
  lcd.backlight();

  for (;;) {
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("MQ2 : ");  lcd.print(mq2);
    lcd.setCursor(0,1); lcd.print("MQ7 : ");  lcd.print(mq7);
    lcd.setCursor(0,2); lcd.print("MQ135: "); lcd.print(mq135);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}


// -------------------------------------------------------
// Task 3 → MQTT (Core 1)
// -------------------------------------------------------
void TaskMQTT(void * parameter) {

  for (;;) {
    if (!client.connected()) {
      reconnectMQTT();
    }

    client.loop();   // WAJIB agar subscribe & publish bekerja

    // --- Kirim data tiap 1 detik ---
    String payload = "{\"MQ2\":" + String(mq2) +
                     ",\"MQ7\":" + String(mq7) +
                     ",\"MQ135\":" + String(mq135) + "}";

    client.publish("sensor/gas", payload.c_str());

    Serial.println("MQTT Sent: " + payload);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}


// -------------------------------------------------------
// SETUP
// -------------------------------------------------------
void setup() {
  Serial.begin(9600);

  pinMode(34, INPUT);
  pinMode(35, INPUT);
  pinMode(25, INPUT);

  // WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println("\nWiFi connected");

  // MQTT setup
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  // ------------------- Task Creation -------------------

  // CORE 0 → Sensor
  xTaskCreatePinnedToCore(
    TaskSensor, "TaskSensor", 4096, NULL, 1, &TaskSensorHandle, 0);

  // CORE 1 → LCD
  xTaskCreatePinnedToCore(
    TaskLCD, "TaskLCD", 4096, NULL, 1, &TaskLCDHandle, 1);

  // CORE 1 → MQTT
  xTaskCreatePinnedToCore(
    TaskMQTT, "TaskMQTT", 4096, NULL, 1, &TaskMQTTHandle, 1);
}

void loop() {
  // Kosong → semua di-handle oleh RTOS
}

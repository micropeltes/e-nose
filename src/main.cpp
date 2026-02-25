#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_ADS1X15.h>

// ===== I2C CONFIG =====
//ESP32C3
#define I2C_SDA  8
#define I2C_SCL  9

//esp32S
// #define I2C_SDA  21
// #define I2C_SCL  22
Adafruit_ADS1115 ads;

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

  const float RLOAD = 3000.0;
  const float VCC = 4.096;  // sesuai GAIN_ONE

  for (;;) {

    int16_t raw_h2s  = ads.readADC_SingleEnded(0);
    int16_t raw_mq135 = ads.readADC_SingleEnded(1);
    int16_t raw_nh3  = ads.readADC_SingleEnded(2);

    float V_h2s  = ads.computeVolts(raw_h2s);
    float V_mq135 = ads.computeVolts(raw_mq135);
    float V_nh3  = ads.computeVolts(raw_nh3);

    // Hitung Rs MQ135
    Rs_mq135 = RLOAD * ((VCC / V_mq135) - 1.0);
    float Rs_kohm = Rs_mq135 / 1000.0;

    Serial.printf(
      "H2S: %.3fV | MQ135: %.3fV Rs: %.2fkΩ | NH3: %.3fV\n",
      V_h2s, V_mq135, Rs_kohm, V_nh3
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
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("TEST");
  if (!ads.begin()) {
    Serial.println("ADS1115 tidak terdeteksi!");
    while (1);
  }
  
  ads.setGain(GAIN_ONE);  // ±4.096V range (default aman)

  // pinMode(34, INPUT);
  // pinMode(35, INPUT);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println("\nWiFi connected!");

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  // ESP32S
  // xTaskCreatePinnedToCore(TaskSensor, "TaskSensor", 4096, NULL, 1, &TaskSensorHandle, 0);
  // xTaskCreatePinnedToCore(TaskLCD, "TaskLCD", 4096, NULL, 1, &TaskLCDHandle, 1);
  // xTaskCreatePinnedToCore(TaskMQTT, "TaskMQTT", 4096, NULL, 1, &TaskMQTTHandle, 1);
  
  // ESP32C3
  xTaskCreate(
      TaskSensor,        // function
      "TaskSensor",      // task name
      4096,              // stack size
      NULL,              // parameter
      2,                 // priority (lebih tinggi)
      &TaskSensorHandle  // task handle
  );

  xTaskCreate(
      TaskLCD,
      "TaskLCD",
      4096,
      NULL,
      1,
      &TaskLCDHandle
  );
  
  xTaskCreate(
      TaskMQTT,
      "TaskMQTT",
      4096,
      NULL,
      1,
      &TaskMQTTHandle
  );
}

void loop() {}

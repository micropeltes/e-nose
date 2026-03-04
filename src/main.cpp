#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;

void setup() {
<<<<<<< Updated upstream
  Serial.begin(9600);
  Wire.begin(I2C_SDA, I2C_SCL);
  
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
=======
  Serial.begin(115200);
  ads.begin();
>>>>>>> Stashed changes
}

void loop() {
  int16_t adc0;
  int16_t adc1;
  int16_t adc2;
  int16_t adc3;

  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  adc2 = ads.readADC_SingleEnded(2);
  adc3 = ads.readADC_SingleEnded(3);

  Serial.print("A0: ");
  Serial.println(adc0+7);
  Serial.print("A1: ");
  Serial.println(adc1+7);
    Serial.print("A2: ");
  Serial.println(adc2+7);
    Serial.print("A3: ");
  Serial.println(adc3+7);
  delay(1000);
}
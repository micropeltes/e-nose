#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;

void setup() {
  Serial.begin(115200);
  ads.begin();
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
  Serial.println(adc0);
  Serial.print("A1: ");
  Serial.println(adc1);
    Serial.print("A2: ");
  Serial.println(adc2);
    Serial.print("A3: ");
  Serial.println(adc3);
  delay(1000);
}
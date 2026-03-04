#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <LiquidCrystal_I2C.h>

Adafruit_ADS1115 ads;

// alamat LCD 0x27, ukuran 16 kolom 2 baris
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  lcd.init();
  lcd.backlight();

  if (!ads.begin()) {
    lcd.setCursor(0,0);
    lcd.print("ADS1115 ERROR");
    while (1);
  }

  lcd.setCursor(0,0);
  lcd.print("ADS1115 Ready");
  delay(2000);
  lcd.clear();
}

void loop() {
  int16_t adc0;
  int16_t adc1;
  int16_t adc2;
  int16_t adc3;

  adc0 = ads.readADC_SingleEnded(0)+7;
  adc1 = ads.readADC_SingleEnded(1)+7;
  adc2 = ads.readADC_SingleEnded(2)+7;
  adc3 = ads.readADC_SingleEnded(3)+7;

  // tampil ke serial
  Serial.print("A0: "); Serial.println(adc0);
  Serial.print("A1: "); Serial.println(adc1);
  Serial.print("A2: "); Serial.println(adc2);
  Serial.print("A3: "); Serial.println(adc3);

  // tampil ke LCD
  lcd.clear();

  lcd.setCursor(0,0);
  lcd.print("A0:");
  lcd.print(adc0);
  lcd.print(" A1:");
  lcd.print(adc1);

  lcd.setCursor(0,1);
  lcd.print("A2:");
  lcd.print(adc2);
  lcd.print(" A3:");
  lcd.print(adc3);

  delay(1000);
}
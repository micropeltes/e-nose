#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <LiquidCrystal_I2C.h>

// I2C defines
Adafruit_ADS1115 ads1;
Adafruit_ADS1115 ads2;
LiquidCrystal_I2C lcd(0x27, 16, 2);
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  lcd.init();
  lcd.backlight();

  // ADS1 ADDR -> GND mq135 dan mems h2s
  lcd.setCursor(0,0);
  if (!ads1.begin(0x48)) {
    lcd.setCursor(0,0);
    lcd.print("ADS1 gagal");
    while (1);
  }
  else{

  }

  // ADS2 ADDR -> SDA mems nh3 dan mics 6814
  if (!ads2.begin(0x4A)) {
    lcd.setCursor(0,1);
    lcd.print("ADS2 gagal");
    while (1);
  }

  lcd.setCursor(0,0);
  lcd.print("ADS1115 Ready");
  delay(2000);
  lcd.clear();    
}

void loop() {

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

  Serial.print("ADS1 CH0: "); Serial.print(adc1_0);
  Serial.print(" | CH1: "); Serial.print(adc1_0);

  Serial.print(" || ADS2 CH0: "); Serial.print(adc1_0);
  Serial.print(" | CH1: "); Serial.println(adc1_0);

  delay(1000);
}
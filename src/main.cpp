#include <Arduino.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);

// Variabel penyimpanan nilai ADC
int mq2 = 0;     // P34 = ADC1_CH6
int mq7 = 0;     // P35 = ADC1_CH7
int mq135 = 0;   // P25 = ADC1_CH8

void setup() {
  lcd.init();
  lcd.backlight();

  // Set pin sebagai input ADC (opsional karena analogRead otomatis)
  pinMode(34, INPUT);
  pinMode(35, INPUT);
  pinMode(25, INPUT);

  lcd.setCursor(0,0);
  lcd.print("ADC Reader Ready");
}

void loop() {
  // Baca nilai ADC
  mq2 = analogRead(34);    // ADC1_CH6
  mq7 = analogRead(35);    // ADC1_CH7
  mq135 = analogRead(25);  // ADC1_CH8

  // Tampilkan ke LCD
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("MQ2 : ");
  lcd.print(mq2);

  lcd.setCursor(0,1);
  lcd.print("MQ7 : ");
  lcd.print(mq7);

  lcd.setCursor(0,2);
  lcd.print("MQ135: ");
  lcd.print(mq135);

  delay(500); // jeda 0.5 detik
}

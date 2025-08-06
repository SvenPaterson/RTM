#include <Arduino.h>
#include <SPI.h>

#define LCD_CS 8
SPISettings lcdSPI(100000, MSBFIRST, SPI_MODE3);

void setup() {
  pinMode(LCD_CS, OUTPUT);
  digitalWrite(LCD_CS, HIGH);
  delay(200);          // ≥100 ms for PIC to boot
  SPI.begin();

  // 1) Clear
  digitalWrite(LCD_CS, LOW);
  SPI.beginTransaction(lcdSPI);
    SPI.transfer(0xFE); SPI.transfer(0x51);
  SPI.endTransaction();
  digitalWrite(LCD_CS, HIGH);
  delay(5);

  // 2) Display ON
  digitalWrite(LCD_CS, LOW);
  SPI.beginTransaction(lcdSPI);
    SPI.transfer(0xFE); SPI.transfer(0x41);
  SPI.endTransaction();
  digitalWrite(LCD_CS, HIGH);

  // 3) Brightness
  digitalWrite(LCD_CS, LOW);
  SPI.beginTransaction(lcdSPI);
    SPI.transfer(0xFE); SPI.transfer(0x53); SPI.transfer(8);
  SPI.endTransaction();
  digitalWrite(LCD_CS, HIGH);

  // 4) Write “HELLO WORLD”
  const char *msg = "HELLO, SPI!";
  digitalWrite(LCD_CS, LOW);
  SPI.beginTransaction(lcdSPI);
    SPI.transfer(0xFE); SPI.transfer(0x45); SPI.transfer(0x00); // home
    for (size_t i = 0; msg[i]; i++) SPI.transfer(msg[i]);
  SPI.endTransaction();
  digitalWrite(LCD_CS, HIGH);
}

void loop() { }

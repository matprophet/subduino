
// Import required libraries
#include <ArducamSSD1306.h>    // Modification of Adafruit_SSD1306 for ESP8266 compatibility
#include <Adafruit_GFX.h>   // Needs a little change in original Adafruit library (See README.txt file)
#include <Wire.h>           // For I2C comm, but needed for not getting compile error

/*
  HardWare I2C pins
  A4   SDA
  A5   SCL
*/

// Pin definitions
#define OLED_RESET  15  // Pin 15 -RESET digital signal
#define THERM_PIN 1
#define UPDATE_INTERVAL 50
int gThermInterval = 0;
int gDispalyInterval = 0;

ArducamSSD1306 display(OLED_RESET); // FOR I2C

void setup(void)
{
  // Start Serial
  Serial.begin(115200);

  // SSD1306 Init
  display.begin();  // Switch OLED
  // Clear the buffer.
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(20, 20);
  display.println("ffffooooooo!");
  display.display();
}

void loop() {
  updateThermistorReadings();
}

void updateThermistorReadings() {
  gThermInterval++;
  if (gThermInterval >= UPDATE_INTERVAL) {
    gThermInterval = 0;

    float R1 = 9200;
    float logR2, R2, TempF, TempC;
    float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

    int Vo = analogRead(THERM_PIN);
    R2 = R1 * (1023.0 / (float)Vo - 1.0);
    logR2 = log(R2);
    TempC = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
    TempC = TempC - 273.15;
    TempF = (TempC * 9.0) / 5.0 + 32.0;

    display.clearDisplay();
    display.setCursor(0, 20);
    display.print("Temp (C):");
    display.println(TempC);
    display.print("Temp (F):");
    display.println(TempF);
    display.display();
  }
}

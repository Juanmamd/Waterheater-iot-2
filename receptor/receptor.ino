
// Project pinout
// D2	- Not used
// D3	- Not used
// D4	- Not used
// D5	- Not used
// D6	- Not used
// D7	- Not used
// D8	- Not used
// D9	- RF24
// D10	- RF24
// D11	- RF24
// D12	- RF24
// D13	- RF24
// A0	- Not used
// A1	- Not used
// A2	- Not used
// A3	- Not used
// A4	- I2C SDA LED
// A5	- I2C SCL LED
// A6	- Not used
// A7	- Not used

// #include <Time.h>
#include <TimeLib.h>
#include <Wire.h>
#include "RTClib.h"

// #include <nRF24L01.h>
// #include <RF24.h>
// #include <RF24_config.h>
#include <SPI.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

const int pinCE = 9;
const int pinCSN = 10;
// RF24 radio(pinCE, pinCSN);

#define LCD_I2C_ADDR    0x3F    // 0x27 in PCF8574 by NXP and Set to 0x3F in PCF8574A
#define RTC_I2C_ADDRESS 0x68


// Single radio pipe address for the 2 nodes to communicate.
const uint64_t pipe = 0xE8E8F0F0E1LL;

char data[16];

LiquidCrystal_I2C lcd(LCD_I2C_ADDR, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
RTC_DS3231 rtc;

char display_line [17];
time_t today;
DateTime dt;

void setup(void)
{
   Serial.begin(115200);
   // radio.begin();
   // radio.openReadingPipe(1,pipe);
   // radio.startListening();
   if (!rtc.begin()) {
     // if (! rtc.isrunning()) {
     //   Serial.println("RTC is NOT running");
     // };
		Serial.println(F("Couldn't find RTC"));
		while (1);
	}

	// Si se ha perdido la corriente, fijar fecha y hora
	if (rtc.lostPower()) {
		// Fijar a fecha y hora de compilacion
    Serial.println("Fijando la fecha de compilacion");
		rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

		// Fijar a fecha y hora específica. En el ejemplo, 21 de Enero de 2016 a las 03:00:00
		// rtc.adjust(DateTime(2016, 1, 21, 3, 0, 0));
	}
  // rtc.adjust(DateTime(2018, 10, 20, 22, 13, 0));
	rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  lcd.begin(16,2,LCD_5x8DOTS);
  lcd.clear();
  lcd.setCursor(0,0);
}

void loop(void)
{
   // if (radio.available())
   // {
   //    // int done = radio.read(data, sizeof data);
   //    Serial.println(data);
   // }
   dt=rtc.now();
   today=dt.unixtime();
   sprintf(display_line, "%02d/%02d   %02d:%02d:%02d",dt.day(),dt.month(),dt.hour(),dt.minute(),dt.second());
   lcd.setCursor(0,0);
   lcd.print(display_line);
   Serial.println(display_line);
   Serial.print(F("Unix time: "));
   Serial.println(dt.unixtime());
   // Serial.println(__DATE__);
   // Serial.println(__TIME__);
   // Serial.println(today);
   delay(500);
}

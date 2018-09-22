// Project pinout
// D2	- Not used
// D3	- Not used
// D3	- Not used
// D4	- Not used
// D5	- Not used
// D6	- Not used
// D7	- Not used
// D8	- Not used
// D9	- Not used
// D10	- Not used
// D11	- Not used
// D12	- Not used
// D13	- Not used
// A0	- Not used
// A1	- Not used
// A2	- Not used
// A3	- Not used
// A4	- I2C SDA
// A5	- I2C SCL
// A6	- Not used
// A7	- Not used

#include <LCD.h>
#include <LiquidCrystal_I2C.h>

#define I2C_ADDR    0x3F    // 0x27 in PCF8574 by NXP and Set to 0x3F in PCF8574A
#define LCD_DISPLAY true

#ifdef LCD_DISPLAY
  LiquidCrystal_I2C lcd(I2C_ADDR, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
  // byte char_ON[] = {  B11111,  B10001,  B10101,  B10101,  B10101,  B10101,  B10001,  B11111  };
  // byte char_OFF[] = {  B11111,  B10001,  B10001,  B10001,  B10001,  B10001,  B10001,  B11111  };
  // const int NUM_DISPLAYS=2;
  // char display_line[2][2][16];
  // int display_active=1;
#endif

void setup(){
  if(LCD_DISPLAY){
    lcd.begin(16,2,LCD_5x8DOTS);
    // lcd.createChar(0, char_OFF);
    // lcd.createChar(1, char_ON);
    lcd.clear();
    // lcd.backlight();
    lcd.setCursor(0,0);
    // lcd.print("Water heater IoT");
    delay(1000);
    lcd.clear();
  };
};



void loop(){
  lcd.setCursor(0,0);
  lcd.print("IoT receiver");

};

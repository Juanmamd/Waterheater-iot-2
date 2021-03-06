// TO DO
// serial commands
// RF commands
// Low power consumption: LCD, RF, Arduino
// send RF data: lines + running send_data
// uptime
// water_temp
// inside_temp
// heater status
// next schedule
// Text [16]
// Use DS3231.h library instead of RTLib.h
// read temperature from RTClib
// Add external temperature sensor for external

// Project pinout
// D2	- Not used
// D3	- Display line button
// D4	- Programmed LED          
// D5	- Heater Relay
// D6	- Heater ON LED
// D7	- Not used
// D8	- Not used
// D9	- Not used
// D10	- Not used
// D11	- Not used
// D12	- Not used
// D13	- DS18B20 Temperature sensor
// A0	- Not used
// A1	- Not used
// A2	- Not used
// A3	- Not used
// A4	- I2C SDA LED + RTC
// A5	- I2C SCL LED + RTC
// A6	- Control Switch
// A7	- Not used

// Stimated current consumption:
// LCD						200 mA
// NRF24L01:				14 mA
// Relay
// Thermpair K MAX6675: 	50 mA
// Arduino:

#include <Time.h>
#include <TimeLib.h>
#include <Wire.h>
#include <DS3231.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <SPI.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#define LCD_I2C_ADDR 0x3F      // 0x27 in PCF8574 by NXP and Set to 0x3F in PCF8574A
#include <OneWire.h>           // For DS18B20
#include <DallasTemperature.h> // For DS18B20

//User controlled variables
const float target_temp PROGMEM = 32; //Desired Temperature
const float hister_temp PROGMEM = 2;  //histeresis
#define STATUS_OFF 0
#define STATUS_ON 1
#define STATUS_PROG 2
byte ProgStatus = STATUS_OFF;

// hardware variables
// const byte stop_button PROGMEM = 2; // Stop programming / manual mode
const byte displayLine PROGMEM = 3; // scroll display lines
const byte ProgrammedLed PROGMEM = 4;
const byte HeaterRelayPin = 5
const byte HeaterPin PROGMEM = 6;   // select the pin for  the LED, relay emulation
const byte controlPin PROGMEM = 13;

const byte pinDatosDQ = 7; // Analog for DS18B20
// Instancia a las clases OneWire y DallasTemperature
OneWire oneWireObjeto(pinDatosDQ);
DallasTemperature sensorDS18B20(&oneWireObjeto);

//System variables
int debug = 1;

int loop_time = 100;                // time to sleep each loop, in msecs
int time_per_degree = 300;          // time to raise 1 degree, in secs, initial value 5 min
byte heater_autoadjust = 1;         // weather time_per_degree is auto adjust from previous data
int max_running_time = 3 * 60 * 60; // maximum running time,default  3 hours

const int eeprom_pos PROGMEM = 100;
// pos 96   time per degree (2 bytes)
// pos 98    111=saved   0=not saved
// pos 99    num_schedules
// pos 100   begin saved schedules
struct dailyprog
{
  byte weekday;  // indexed by week[]
  byte use_time; // time from 4 am to 23 am, 5 minutes interval.
  byte hour;
  byte minute;
};
dailyprog schedule[14];     // two schedules per day.
char week[] = {" LMXJVSD"}; // Week begins in 1, Monday
byte num_schedules;
byte max_schedules = 14;

//Internal variables
// byte stop_programm = 0;      // stop schedule running
byte heater_on = 0;          // heater status
int count = 1;               // Loop count
float water_temp;            // current water temperature and target temperature
float init_temp, final_temp; // controls autoadjustment
int time_to_warm;            // time left to reach target_temp, in secs
int time_to_sched, time_in_sched;
time_t t_now; // date and time. time left for next use
DateTime next_schedule;
time_t init_time, fin_time, running_time; // running time control
DateTime boot_time;
// const uint64_t pipe PROGMEM = 0xE8E8F0F0E1LL;
byte perform_adjust = 0;
int controlVal = 0; // ADC read, ON,OFF,PROG Switch

LiquidCrystal_I2C lcd(LCD_I2C_ADDR, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
byte char_ON[] = {B11111, B10001, B10101, B10101, B10101, B10101, B10001, B11111};
byte char_OFF[] = {B11111, B10001, B10001, B10001, B10001, B10001, B10001, B11111};
const byte NUM_DISPLAYS PROGMEM = 3; // Num of rotating screens
char display_line[4][17];            // LCD lines size
byte display_line_i = 1;
byte display_active = 1;
// long int uptime=0;
long int uptime;
int debouncing = 1; // minimum time between button Pressed
time_t last_interrupt = 0;

// RTC_DS3231 rtc;
DS3231 clock;
RTClib rtc;
DateTime dt;
float clock_temp;

void setup()
{
  Serial.begin(115200);
  lcd.begin(16, 2, LCD_5x8DOTS);
  lcd.createChar(0, char_OFF);
  lcd.createChar(1, char_ON);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Water heater IoT"));

  //  unsave_schedules(); // Set eeprom_pos -2 to 0
  load_schedules(schedule);
  show_schedules(schedule);

  pinMode(HeaterPin, OUTPUT);
  pinMode(ProgrammedLed, OUTPUT);
  pinMode(HeaterRelayPin, OUTPUT);

  sensorDS18B20.begin();
  water_temp = get_watertemp();
  if (water_temp == -127)
  {
    Serial.println("Sensor DS18B20 no found");
    lcd.setCursor(0, 0);
    lcd.print(F("Temperat. error"));
    lcd.setCursor(0, 1);
    lcd.print(F("DS18B20 missing"));
    while (1)
    {
      digitalWrite(HeaterPin, HIGH);
      delay(300);
      digitalWrite(HeaterPin, LOW);
      delay(300);
    }
  }

  // Initialize programm led
  digitalWrite(ProgrammedLed, HIGH);

  //  clock.setClockMode(0);
  //  clock.setYear(18);
  //  clock.setMonth(12);
  //  clock.setDate(15);
  //  clock.setHour(21);
  //  clock.setMinute(42);
  //  clock.setSecond(00);
  //  clock.setDoW(6);

  dt = rtc.now();
  boot_time = dt; // for uptime calculation
  if (debug) print_time(dt);

  char display_line_t[17];

  attachInterrupt(digitalPinToInterrupt(displayLine), display_show_line, RISING); // Show next lcd line button
  delay(1000);
};

void stop_heater()
{
}

void loop()
{
  controlVal = analogRead(controlPin);
  if (debug)
  {
    Serial.print("Switch value: ");
    Serial.println(controlVal);
  }
  if (controlVal < 150)
  {
    ProgStatus = STATUS_OFF;
    // stop_programm_int();
    power_off_heater();           // Stops heater inmediately
    digitalWrite(ProgrammedLed, LOW);
  }
  else if (controlVal > 650)
  {
    ProgStatus = STATUS_ON;
    // power_on_heater();         // Will check conditions later for power on heater
    digitalWrite(ProgrammedLed, LOW);
  }
  else
  {
    ProgStatus = STATUS_PROG;
    digitalWrite(ProgrammedLed, HIGH);
  }
  clock_temp = clock.getTemperature();  // Internal clock temperature
  dt = rtc.now();
  t_now = dt.unixtime();
  uptime = t_now - boot_time.unixtime();
  water_temp = get_watertemp();
  time_to_warm = (target_temp - water_temp) * time_per_degree; // current time to warm water at this temperature, in secs
  next_schedule = get_next_schedule(schedule);                 // Get next time when water has to be warmed, in secs
  time_to_sched = next_schedule.unixtime() - t_now;
  // time_to_sched=next_schedule - t_now;
  if (debug)
  {
    Serial.print(F("Uptime: "));
    print_time(uptime);
    Serial.print(F("Next:"));
    print_time(next_schedule);
    Serial.println();
    Serial.print(F("Now:"));
    print_time(dt);
    Serial.println();
    Serial.println(F("Time to sched: "));
  };
  if (heater_on) running_time = t_now - init_time; // Calculate running time for displaying
  // if (water_temp < target_temp && time_to_warm > time_to_sched && t_now < next_schedule && !stop_programm) {
  if ((water_temp < target_temp &&
       time_to_warm > time_to_sched &&
       t_now < next_schedule.unixtime() &&
       ProgStatus == STATUS_PROG ||
      //  !stop_programm) ||
      (water_temp < target_temp && ProgStatus == STATUS_ON))
  {
    if (!heater_on)
    {
      power_on_heater();
      init_time = t_now;
      init_temp = water_temp;
    };
  }
  else if (water_temp < (target_temp - hister_temp))
  { // histeresis control
    if (debug) Serial.println(F("En espera"));
    if (heater_on)
    {
      power_off_heater();
      // final_temp = water_temp;
      if (perform_adjust)
        time_per_degree = running_time / (water_temp - init_temp); // auto-adjust time per degree
      // Save time_per_degre on eeprom
      log_session(); // Send this sesion over RF24
    };
  };
  display(water_temp, target_temp, running_time, dt, next_schedule);
  delay(loop_time);
  count++;
};

void unsave_schedules()
{
  EEPROM.put(eeprom_pos - 2, 0);
}

void save_schedules()
{
  byte mem_count = 0;
  byte next_pos = eeprom_pos;
  // Serial.println(F("save_schedules"));
  EEPROM.write(eeprom_pos - 2, 111);
  EEPROM.write(eeprom_pos - 1, num_schedules);
  while (mem_count < num_schedules)
  {
    next_pos = mem_count * sizeof(dailyprog) + eeprom_pos;
    EEPROM.put(next_pos, schedule[mem_count]);
    if (debug)
    {
      Serial.print(F("Sched write: "));
      Serial.print(schedule[mem_count].weekday);
      Serial.print(F(" / "));
      Serial.println(schedule[mem_count].use_time);
    }
    mem_count++;
  }
  if (debug)
  {
    Serial.println(F("Saved data to EEPROM"));
  }
}

void load_schedules(struct dailyprog *schedule)
{ // TODO
  byte mem_count = 0;
  byte next_pos = eeprom_pos;
  Serial.println(F("load_schedules"));
  if (EEPROM.read(eeprom_pos - 2) == 111)
  { // Read if EEPROM has saved data. 1 byte.
    if (debug)
    {
      Serial.println(F("Loading data from EEPROM"));
    }
    time_per_degree = EEPROM.read(eeprom_pos - 4);
    num_schedules = EEPROM.read(eeprom_pos - 1); // Read num_schedules . 1 byte.
    while (mem_count < num_schedules)
    {
      next_pos = mem_count * sizeof(dailyprog) + eeprom_pos;
      schedule[mem_count] = EEPROM.get(next_pos, schedule[mem_count]);
      if (debug)
      {
        Serial.print(F("Sched read: "));
        Serial.print(schedule[mem_count].weekday);
        Serial.print(F(" / "));
        Serial.println(schedule[mem_count].use_time);
      }
      mem_count++;
    }
    // schedule=EEPROM.get(eeprom_pos,dailyprog [14]);
  }
  else
  {
    // Default schedule
    schedule[0].weekday = 7;
    schedule[0].use_time = 75; // 10:15 am.  (10-4)*12 + 15/5=75  75*5=375  375/60 = 6,25 6h 15min + 4.00 = 10,15
    schedule[0].hour = 10;
    schedule[0].minute = 15;
    schedule[1].weekday = 7;
    schedule[1].use_time = 38;
    schedule[1].hour = 7;
    schedule[1].minute = 0;
    schedule[2].weekday = 1;
    schedule[2].use_time = 39;
    schedule[2].hour = 7;
    schedule[2].minute = 0;
    schedule[3].weekday = 2;
    schedule[3].use_time = 40; // 7:00    7-4=3 3*12=36
    schedule[3].hour = 7;
    schedule[3].minute = 0;
    schedule[4].weekday = 3;
    schedule[4].use_time = 41; // 7:00    7-4=3 3*12=36
    schedule[4].hour = 7;
    schedule[4].minute = 0;
    schedule[5].weekday = 4;
    schedule[5].use_time = 42; // 7:00    7-4=3 3*12=36
    schedule[5].hour = 7;
    schedule[5].minute = 0;
    schedule[6].weekday = 5;
    schedule[6].use_time = 43; // 7:00    7-4=3 3*12=36
    schedule[6].hour = 7;
    schedule[6].minute = 0;
    schedule[7].weekday = 6;
    schedule[7].use_time = 44; // 7:00    7-4=3 3*12=36
    schedule[7].hour = 7;
    schedule[7].minute = 0;
    num_schedules = 8;
    save_schedules();
  }
}

int time_to_minutes(time_t inputtime)
{
  int return_minutes;
  return_minutes = day(inputtime) * 1440 + hour(inputtime) * 60 + minute(inputtime);
  return return_minutes;
};

void display_show_line()
{
  if (debug)
    Serial.println(F("Interrupt display line"));
  if (last_interrupt + debouncing < t_now)
  {
    display_line_i++;
    if (display_line_i > NUM_DISPLAYS)
      display_line_i = 1;
    Serial.print(F("Displaying line "));
    Serial.println(display_line_i);
    // delay(200);
    last_interrupt = t_now;
    if (debug)
    {
      Serial.print("Last interrupt: ");
      Serial.println(last_interrupt);
    }
  };
};

// void stop_programm_int()
// {
//   if (last_interrupt + debouncing < t_now)
//   {
//     stop_programm = !stop_programm;
//     digitalWrite(ProgrammedLed, !stop_programm);
//     Serial.println(F("Stop BUTTON Pressed"));
//     last_interrupt = t_now;
//   };
// };

void show_schedules(struct dailyprog *schedule)
{
  int sch_i;
  int hour, min;
  for (sch_i = 0; sch_i < num_schedules; sch_i++)
  {
    Serial.print(F("Show schedule: "));
    Serial.print(week[schedule[sch_i].weekday]);
    Serial.print(F(" - "));
    hour = schedule[sch_i].use_time / 12 + 4;
    min = schedule[sch_i].use_time % 12 * 5;
    Serial.print(hour);
    Serial.print(F(":"));
    Serial.println(min);
  }
};

float get_watertemp()
{                                       //Implement according to hardware sensor
  float sensortemp;
  sensorDS18B20.requestTemperatures();
  sensortemp = sensorDS18B20.getTempCByIndex(0);
  if (debug)
  {
    Serial.print(F("Temperature: "));
    Serial.println(sensortemp);
  };
  return sensortemp;
};

DateTime get_next_schedule(struct dailyprog *sched)
{
  DateTime return_time;
  int target_weekday = clock.getDoW();
  int sch_i = 0;
  int time_diff = 14400; // Ten days in minutes
  int time1, time2, sch_found;
  // time_t t_next;
  boolean next_found = false;


  time1 = clock.getDoW() * 1440 + dt.hour() * 60 + dt.minute(); // Now in minutes since past sunday
  // if (debug)
  // {
  //   Serial.print(F("time1: "));
  //   Serial.print(F(" / "));
  //   Serial.print(clock.getDoW());
  //   Serial.print(F(" / "));
  //   Serial.print(dt.hour());
  //   Serial.print(F(" / "));
  //   Serial.println(dt.minute());
  // }
  while (sch_i < num_schedules)
  {
    // time2 = sched[sch_i].weekday * 1440 + (sched[sch_i].use_time / 12 + 4) * 60 + sched[sch_i].use_time % 12 * 5;
    // time2= sched[sch_i].weekday * 1440 + sched[sch_i].hour * 6 + sched[sch_i].minute;
    if(sched[schi].weekday < targe_weekday){  // looking into next week
      time2 = (sched[sch_i].weekday+7)*1440 + sched[sch_i].hour*60 + sched[sch_i];
    }else{                                    // lookin into this week
      time2 = sched[sch_i].weekday*1440 + sched[sch_i].hour*60 + sched[sch_i];
    }
    // t_next = 
    // if (debug)
    // {
    //   Serial.print(F("Sched comparing: "));
    //   Serial.print(time1);
    //   Serial.print(F(" -> "));
    //   Serial.println(time2);
    // }
    if (time2 - time1 < time_diff && time2 > time1)
    { // search for minimum difference (next schedule)
      time_diff = time2 - time1;
      sch_found = sch_i;
      Serial.print(F("Best found: "));
      Serial.println(sch_i);
    };
    sch_i++;
  };
  return_time = {dt.year(), dt.month(),
                 dt.day() + sched[sch_found].weekday - clock.getDoW(),
                 sched[sch_found].hour,
                 sched[sch_found].minute, 0};
  return return_time;
};

void print_time(time_t sched)
{
  if (debug)
  {
    Serial.print(F("print_time: "));
    Serial.print(sched);
    Serial.println();
  }
  if (year(sched) > 1970)
  {
    Serial.print(day(sched));
    Serial.print(F("/"));
    Serial.print(month(sched));
    Serial.print(F("/"));
    Serial.print(year(sched));
    Serial.print(F(" "));
  };
  Serial.print(hour(sched));
  Serial.print(F(":"));
  Serial.print(minute(sched));
  Serial.print(F(":"));
  Serial.print(second(sched));
};

void print_time(DateTime dt)
{
  Serial.println(F("print_time DateTime"));
  Serial.print(dt.day());
  Serial.print(F("/"));
  Serial.print(dt.month());
  Serial.print(F("/"));
  Serial.print(dt.year());
  Serial.print(F(" "));
  Serial.print(dt.hour());
  Serial.print(F(":"));
  Serial.print(dt.minute());
  Serial.print(F(":"));
  Serial.print(dt.second());
};

void power_on_heater()
{ 
  Serial.println(F("Encendiendo calentador"));
  heater_on = 1;
  digitalWrite(HeaterPin, HIGH);
  digitalWrite(HeaterRelayPin, HIGH);
};

void power_off_heater()
{ 
  Serial.println(F("Apagando calentador"));
  heater_on = 0;
  digitalWrite(HeaterPin, LOW);
  digitalWrite(HeaterRelayPin, LOW);
};

void log_session(){}; // TODO

void display(float temp1, float temp2, long int r_time, DateTime today, DateTime next)
{
  char buff1[6], buff2[6];
  Serial.println(temp1);
  Serial.println(temp2);
  dtostrf(temp1, 4, 2, buff1);
  dtostrf(temp2, 4, 2, buff2);
  // sprintf(display_line [0], "%02d/%02d    %02d:%02d  ",day(today),month(today),hour(today),minute(today));
  // sprintf(display_line [1], "T: %s->%s", buff1, buff2);
  // sprintf(display_line [2], "%02d/%02d    %02d:%02d  ",day(next),month(next),hour(next),minute(next));
  sprintf(display_line[0], "%02d/%02d    %02d:%02d  ", today.day(), today.month(), today.hour(), today.minute());
  sprintf(display_line[1], "T: %s->%s", buff1, buff2);
  // sprintf(display_line [1], "T: %.2f->%.2f", temp1, temp2);
  sprintf(display_line[2], "%02d/%02d    %02d:%02d  ", next.day(), next.month(), next.hour(), next.minute());
  // sprintf(display_line [3], "%02d/%02d -- %02d:%02d  ",dt.day(), dt.month(), dt.hour(), dt.minute());
  dtostrf(clock_temp, 4, 2, buff1);
  sprintf(display_line[3], "Internal: %s   ", buff1);
  if (display_active)
  { // Display Active
    lcd.setCursor(0, 0);
    lcd.print(display_line[0]);
    Serial.println(display_line[0]);
    lcd.setCursor(0, 1);
    lcd.print(display_line[display_line_i]);
    Serial.println(display_line[display_line_i]);
    lcd.setCursor(15, 0);
    lcd.write(heater_on);
  }
  else
  { // No display available
    Serial.print(F("Water temperature: "));
    Serial.println(temp1, DEC);
    Serial.print(F("Target temperature: "));
    Serial.println(temp2);
    Serial.print(F("Heater is: "));
    if (heater_on)
      Serial.println(F("ON"));
    else
      Serial.println(F("OFF"));
    Serial.print(F("Running time: "));
    Serial.println(r_time);
    Serial.print(F("Date/Time: "));
    // Serial.println(today);
    print_time(today);
    Serial.print(F("Time to target: "));
    // Serial.println(next-today);
    Serial.println(next.unixtime() - today.unixtime());
  };
  if (debug)
  {
    Serial.print(F("Water temperature: "));
    Serial.println(temp1, DEC);
    Serial.print(F("Target temperature: "));
    Serial.println(temp2);
    Serial.print(F("Heater is: "));
    if (heater_on)
      Serial.println(F("ON"));
    else
      Serial.println(F("OFF"));
    Serial.print(F("Running time: "));
    Serial.println(r_time);
    Serial.print(F("Date/Time: "));
    // Serial.println(today);
    print_time(today);
    Serial.print(F("Time to target: "));
    Serial.println(next.unixtime() - today.unixtime());
    Serial.print(F("Performance: "));
    Serial.print(time_per_degree);
    Serial.println(F(" secs per degree"));
    Serial.print(F("Programmed: "));
    Serial.println(!stop_programm);
    Serial.print(F("Next: "));
    // Serial.println(next);
    print_time(next);
  };
};
//
// // Implementation due to Tomohiko Sakamoto
// byte DayOfWeek(int y, byte m, byte d) {   // y > 1752, 1 <= m <= 12
//   static int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
//
//   y -= m < 3;
//   return ((y + y/4 - y/100 + y/400 + t[m-1] + d) % 7) + 1; // 01 - 07, 01 = Sunday
// }

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
// save/restore schedules and variables from eeprom

// Project pinout
// D2	- Stop program button
// D3	- Heater LED
// D4	- Programmed LED
// D5	- Display control button
// D6	- Display line button
// D7	- DS18B20 Temperature sensor
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
// A4	- I2C SDA LED + RTC
// A5	- I2C SCL LED + RTC 
// A6	- Not used
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
// #include <max6675.h>
#include <avr/pgmspace.h>


#include <SPI.h>
#include "RF24.h"
RF24 radio(9,10);

#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#define LCD_I2C_ADDR    0x3F    // 0x27 in PCF8574 by NXP and Set to 0x3F in PCF8574A

#include <OneWire.h>              // For DS18B20
#include <DallasTemperature.h>    // For DS18B20
// Thermpair
// Pin donde se conecta el bus 1-Wire
const int pinDatosDQ = 7; // for DS18B20
// Instancia a las clases OneWire y DallasTemperature
OneWire oneWireObjeto(pinDatosDQ);
DallasTemperature sensorDS18B20(&oneWireObjeto);

//User controlled variables
const float target_temp PROGMEM =32;  //Desired Temperature
const float hister_temp PROGMEM =2;   //histeresis
// float target_temp = 35;

// hardware variables
const int sensorPin PROGMEM  =  A0;     // select the input  pin for the potentiometer
const int HeaterPin PROGMEM  =  6;      // select the pin for  the LED, relay emulation
const byte stop_button PROGMEM = 2;     // Stop programming / manual mode
const byte displayPin PROGMEM=5;        // Display config
const byte displayLine PROGMEM=3;       // scroll display lines
const int ProgrammedLed PROGMEM=4;

//System variables
int debug=0;
int loop_time = 100;  					     // time to sleep earch loop, in secs
int time_per_degree = 60; 			     // time to raise 1 degree, in secs, initial value
int heater_autoadjust = 1;				   // weather time_per_degree is auto adjust from previous data
int max_running_time = 3 * 60 *60 ;			 // maximum running time

const int eeprom_pos PROGMEM =100;
                        // pos 97   time per degree
                        // pos 98    111=saved   0=not saved
                        // pos 99    num_schedules
                        // pos 100   begin saved schedules
struct dailyprog {
  byte weekday;  // indexed by week[]
  byte use_time;  // time from 4 am to 23 am, 5 minutes interval.
} ;
dailyprog schedule[14]; // two schedules per day.
char week[]={" DLMXJVS"}; // Week begins in 1, sunday
int num_schedules;

//Internal variables
int stop_programm = 0;					    // stop schedule running
int heater_on=0;						          // heater status
int sensorValue=0;  // variable to  store  the value  coming  from  the sensor
int count=1;          // Loop count
float water_temp;			  // current water temperature and target temperature
float init_temp, final_temp;			    // controls autoadjustment
int time_to_warm;	  // time left to reach target_temp, in secs
int time_to_sched, time_in_sched; //
time_t date, next_schedule, t_now, boot_time;      // date and time. time left for next use
time_t init_time, fin_time, running_time;	// running time control
const uint64_t pipe PROGMEM = 0xE8E8F0F0E1LL;
int perform_adjust=0;

LiquidCrystal_I2C lcd(LCD_I2C_ADDR, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
byte char_ON[] = {  B11111,  B10001,  B10101,  B10101,  B10101,  B10101,  B10001,  B11111  };
byte char_OFF[] = {  B11111,  B10001,  B10001,  B10001,  B10001,  B10001,  B10001,  B11111  };
const int NUM_DISPLAYS PROGMEM =3; // Num of rotating screens
char display_line[4][17]; // LCD lines size
int display_line_i=1;
int display_active=1;
long int uptime=0;
int debouncing=300; // min time between button Pressed
time_t last_interrupt=0;

void setup() {
  schedule[0].weekday=1;
  schedule[0].use_time=75;  // 10:15 am.  (10-4)*12 + 15/5=75  75*5=375  375/60 = 6,25 6h 15min + 4.00 = 10,15
  schedule[1].weekday=1;
  schedule[1].use_time=38;   // 7:00    7-4=3 3*12=36
  schedule[2].weekday=2;
  schedule[2].use_time=39;   // 7:05    7-4=3 3*12=36
  schedule[3].weekday=3;
  schedule[3].use_time=40;   // 7:10    7-4=3 3*12=36
  schedule[4].weekday=4;
  schedule[4].use_time=41;   // 7:20    7-4=3 3*12=36
  schedule[5].weekday=5;
  schedule[5].use_time=42;   // 7:25    7-4=3 3*12=36
  schedule[6].weekday=6;
  schedule[6].use_time=43;   // 7:30    7-4=3 3*12=36
  schedule[7].weekday=7;
  schedule[7].use_time=198;  // 20:30 am.  (20-4)*12=192 + 30/5=6 198 75*5=375  375/60 = 6,25 6h 15min + 4.00 = 10,15
  num_schedules=8;

  // Prototype
  pinMode(HeaterPin,OUTPUT);
  pinMode(ProgrammedLed, OUTPUT);
  // pinMode(displayPin,INPUT);
  // attachInterrupt(digitalPinToInterrupt(displayPin),display_control_int, RISING);
  attachInterrupt(digitalPinToInterrupt(stop_button),stop_programm_int, RISING); // Stop programm interrupt
  attachInterrupt(digitalPinToInterrupt(displayLine),display_show_line,RISING);
  Serial.begin(115200);
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(pipe);
  radio.openReadingPipe(1,pipe);
  sensorDS18B20.begin();
  date = get_date(); // Ask for remote date
  set_date(date);
  // show_schedules(schedule);
  load_schedules(schedule);

  digitalWrite(ProgrammedLed,!stop_programm);
  lcd.begin(16,2,LCD_5x8DOTS);
  lcd.createChar(0, char_OFF);
  lcd.createChar(1, char_ON);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Water heater IoT"));
  delay(1000);
  boot_time=now();
};

void loop(){
  // time_t mynext_schedule;

 // if(digitalRead(stop_button) == HIGH) stop_programm_int();
 // if(digitalRead(displayPin) == HIGH) display_control_int();
 // if(digitalRead(displayLine) == HIGH) display_show_line();

  t_now=now();
  uptime=t_now-boot_time;
  water_temp = get_watertemp();
  time_to_warm = ( target_temp - water_temp) * time_per_degree;		// current time to warm water at this temperature, in secs
  next_schedule = get_next_schedule(schedule);				// Get next time when water has to be warmed, in secs
  time_to_sched=next_schedule - t_now;
  if(debug){
    Serial.println(F("Next:"));
    print_time(next_schedule);
    Serial.println(F("Now:"));
    print_time(t_now);
    Serial.print(F("Time to sched: "));
    Serial.println(time_to_minutes(time_to_sched));
  };
  if (heater_on) running_time = t_now - init_time;		// Calculate running time for displaying
  if (water_temp < target_temp && time_to_warm > time_to_sched && t_now < next_schedule && !stop_programm) {
    Serial.println(F("Calentando"));
    if (!heater_on) {
      power_on_heater();
      init_time = t_now;
      init_temp = water_temp;
    };
  } else if(water_temp < (target_temp - hister_temp)) {  // histeresis control
    if(debug)Serial.println(F("En espera"));
    if (heater_on) {
      power_off_heater();
      final_temp = water_temp;
      if (perform_adjust) time_per_degree = running_time / (final_temp - init_temp); // auto-adjust time per degree
      // Save time_per_degre on eeprom
      log_session(); // Send this sesion over RF24
    };
  };
  display(heater_on, water_temp, target_temp, running_time, now(), next_schedule);
  delay(loop_time);
  count++;
};

int time_to_minutes(time_t inputtime){
  int return_minutes;
  return_minutes=day(inputtime)*1440 + hour(inputtime)*60 + minute(inputtime);
  return return_minutes;
};

void display_show_line(){
  if(last_interrupt+debouncing < t_now){
    display_line_i++;
    if(display_line_i>NUM_DISPLAYS)display_line_i=1;
    Serial.print(F("Displaying line "));
    Serial.println(display_line_i);
    // delay(200);
    last_interrupt=t_now;
  };
};

// void display_control_int(){
//   display_active++;
//   if(display_active>2)display_active=0;
//   Serial.println(F("Display buttons pressed: "));
//   Serial.println(display_active);
//   switch(display_active){
//     case 0:
//       lcd.noDisplay();
//       ;
//     case 1:
//       lcd.display();
//       lcd.noBacklight();
//       ;
//     case 2:
//       lcd.display();
//       lcd.backlight();
//       ;
//   };
//   delay(50);
// };

int send_data(){
  radio.stopListening();
  Serial.println(F("Now sending"));
  unsigned long start_time = micros();                             // Take the time, and send it.  This will block until complete
/* Enviar struct */
   if (!radio.write( &start_time, sizeof(unsigned long) )){
     Serial.println(F("Send failed"));
   };
  radio.startListening();                                    // Now, continue listening
  unsigned long started_waiting_at = micros();               // Set up a timeout period, get the current microseconds
  boolean timeout = false;                                   // Set up a variable to indicate if a response was received or not
  while ( ! radio.available() ){                             // While nothing is received
    if (micros() - started_waiting_at > 200000 ){            // If waited longer than 200ms, indicate timeout and exit while loop
        timeout = true;
        break;
    };
  };
  if ( timeout ){                                             // Describe the results
      Serial.println(F("Failed, response ACK timed out."));
  }else{
      unsigned long got_time;                                 // Grab the response, compare, and send to debugging spew
      radio.read( &got_time, sizeof(unsigned long) );
      unsigned long end_time = micros();
      // Spew it
      Serial.print(F("Sent "));
      Serial.print(start_time);
      Serial.print(F(", Got response "));
      Serial.print(got_time);
      Serial.print(F(", Round-trip delay "));
      Serial.print(end_time-start_time);
      Serial.println(F(" microseconds"));
  };
  // Try again 1s later
  delay(3000);
};

void stop_programm_int(){
  if(last_interrupt+debouncing < t_now){
    stop_programm=!stop_programm;
    digitalWrite(ProgrammedLed,!stop_programm);
    Serial.println(F("Stop BUTTON Pressed"));
    last_interrupt=t_now;
  };
};

int set_date(time_t date){  // Need to implement an NTP-like using RF24 get_date()
  setTime(23,05,00,15,10 ,2018);
  };

time_t get_date(){ // TODO
  // Gets date and time from external source
  };

int load_schedules(struct dailyprog *schedule){}; // TODO

int show_schedules(struct dailyprog *schedule){
    int sch_i;
    int hour,min;
    for(sch_i=0;sch_i<num_schedules;sch_i++){
      Serial.println(F("Schedule loop"));
      Serial.print(week[schedule[sch_i].weekday]);
      Serial.print(" - ");
      hour=schedule[sch_i].use_time/12 + 4;
      min=schedule[sch_i].use_time%12 * 5;
      Serial.print(hour);
      Serial.print(":");
      Serial.println(min);
    }
};

float get_watertemp(){  //Implement according to hardware sensor
    float sensortemp;
    sensorDS18B20.requestTemperatures();
    sensortemp=sensorDS18B20.getTempCByIndex(0);
    if(debug){
      Serial.print(F("Temperature: "));
      Serial.println(sensortemp);
    };
    // Serial.println("");
    return sensortemp;
};

time_t get_next_schedule(struct dailyprog *sched){ // TODO
  time_t return_time;
  tmElements_t timeElements;
  int target_weekday=weekday(t_now);
  int sch_i=0;
  int time_found=14400; // Ten days in minutes
  int time1, time2, sch_found;
  boolean next_found=false;

  time1=weekday(t_now)*1440 + hour(t_now)*60 + minute(t_now);  // Now in minutes since past sunday
  while(sch_i < num_schedules){
      time2=sched[sch_i].weekday*1440 + (sched[sch_i].use_time/12 + 4)*60 + sched[sch_i].use_time%12 * 5;
      if(time2-time1<time_found && time2-time1>0){  // search for minimum difference (next schedule)
        time_found=time2-time1;
        sch_found=sch_i;
      };
    sch_i++;
  };
  timeElements={0,sched[sch_found].use_time%12 * 5, sched[sch_found].use_time/12 + 4,
                  sched[sch_found].weekday,day(t_now)+(sched[sch_found].weekday-weekday(t_now)),
                  month(t_now),year(t_now)-1970};
  return_time=makeTime(timeElements);
  return return_time;
  };

void print_time(time_t sched){
  if(year(sched)>1970){
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
  Serial.println(second(sched));
};

int power_on_heater(){ // TODO Implement relay control
  Serial.println(F("Encendiendo calentador"));
  heater_on=1;
  digitalWrite(HeaterPin, HIGH);
  };

int power_off_heater(){ // TODO Implement relay control
  Serial.println(F("Apagando calentador"));
  heater_on=0;
  digitalWrite(HeaterPin, LOW);
  };

int log_session(){}; // TODO

int display(int h_status, float temp1, float temp2, int r_time, time_t today, time_t next){
  char buff1[5], buff2[5];
  dtostrf(temp1,2,2,buff1);
  dtostrf(temp2,2,2,buff2);
  sprintf(display_line [0], "%02d/%02d    %02d:%02d  ",day(today),month(today),hour(today),minute(today));
  sprintf(display_line [1], "T: %s->%s", buff1, buff2);
  sprintf(display_line [2], "%02d/%02d    %02d:%02d  ",day(next),month(next),hour(next),minute(next));
  sprintf(display_line [3], "Linea 3         ");
  if(display_active){ // Display Active
    lcd.setCursor(0,0);
    lcd.print(display_line[0]);
    lcd.setCursor(0,1);
    lcd.print(display_line[display_line_i]);
  }else{ // No display available
    Serial.print(F("Water temperature: "));
    Serial.println(temp1,DEC);
    Serial.print(F("Target temperature: "));
    Serial.println(temp2);
    Serial.print(F("Heater is: "));
    if(h_status)Serial.println(F("ON")); else Serial.println(F("OFF"));
    Serial.print(F("Running time: "));
    Serial.println(r_time);
    Serial.print(F("Date/Time: "));
    Serial.println(today);
    Serial.print(F("Time to target: "));
    Serial.println(next-today);
  };
  if(debug){
    Serial.print(F("Water temperature: "));
    Serial.println(temp1,DEC);
    Serial.print(F("Target temperature: "));
    Serial.println(temp2);
    Serial.print(F("Heater is: "));
    if(h_status)Serial.println(F("ON")); else Serial.println(F("OFF"));
    Serial.print(F("Running time: "));
    Serial.println(r_time);
    Serial.print(F("Date/Time: "));
    Serial.println(today);
    Serial.print(F("Time to target: "));
    Serial.println(next-today);
    Serial.print(F("Performance: "));
    Serial.print(time_per_degree);
    Serial.println(F(" secs per degree"));
    Serial.print(F("Programmed: "));
    Serial.println(!stop_programm);
    Serial.print(F("Next: "));
    Serial.println(next);
  };
};

// TO DO
// check maximum working time.
// serial commands
// several displays screens
// Low power consumption: LCD, RF, Arduino

// Project pinout
// D2	- Stop program button
// D3	- Heater LED
// D4	- Programmed LED
// D5	- Display control
// D6	- Display line
// D7	- DS18B20 Temperature sensor
// D8	- Not used
// D9	- RF24
// D10	- RF24
// D11	- RF24
// D12	- RF24
// D13	- RF24
// A0	- sensor pin?
// A1	- Not used
// A2	- Not used
// A3	- Not used
// A4	- I2C SDA
// A5	- I2C SCL
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
#include <max6675.h>

#define I2C_ADDR    0x3F    // 0x27 in PCF8574 by NXP and Set to 0x3F in PCF8574A

// #ifdef RF24
  #include <SPI.h>
  #include "RF24.h"
  #include <LCD.h>
  #include <LiquidCrystal_I2C.h>
  RF24 radio(9,10);
// #endif

#include <OneWire.h>
#include <DallasTemperature.h>

// Thermpair
// Pin donde se conecta el bus 1-Wire
const int pinDatosDQ = 7;
// Instancia a las clases OneWire y DallasTemperature
OneWire oneWireObjeto(pinDatosDQ);
DallasTemperature sensorDS18B20(&oneWireObjeto);

//User controlled variables
float target_temp=35;

// hardware variables
int sensorPin  =  A0;     // select the input  pin for the potentiometer
int HeaterPin  =  3;   // select the pin for  the LED, relay emulation
const byte stop_button = 2; // Stop programming / manual mode
const byte displayPin=5; // Display config
const byte displayLine=6; // scroll display lines
int ProgrammedLed=4;

//System variables
int debug=0;
int loop_time = 500;  					      // time to sleep earch loop, in secs
int time_per_degree = 60; 			    // time to raise 1 degree, in secs, initial value
int heater_autoadjust = 0;				      // weather time_per_degree is auto adjust from previous data
int max_running_time = 4 * 60;			// 4 mins maximum running time, default = 4 hours

int eeprom_pos=100;     // pos 98    1=saved   0=not saved
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
time_t date, next_schedule, t_now;      // date and time. time left for next use
time_t init_time, fin_time, running_time;	// running time control
const uint64_t pipe = 0xE8E8F0F0E1LL;
int perform_adjust=0;

// #ifdef LCD_DISPLAY
  LiquidCrystal_I2C lcd(I2C_ADDR, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
  byte char_ON[] = {  B11111,  B10001,  B10101,  B10101,  B10101,  B10101,  B10001,  B11111  };
  byte char_OFF[] = {  B11111,  B10001,  B10001,  B10001,  B10001,  B10001,  B10001,  B11111  };
  const int NUM_DISPLAYS=3; // Num of rotating screens
  char display_line[4][17]; // LCD lines size
  int display_line_i=1;
  int display_active=1;

// #endif

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
  attachInterrupt(digitalPinToInterrupt(displayPin),display_control_int, RISING);
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
  show_schedules(schedule);
  load_schedules(schedule);

  digitalWrite(ProgrammedLed,!stop_programm);
  lcd.begin(16,2,LCD_5x8DOTS);
  lcd.createChar(0, char_OFF);
  lcd.createChar(1, char_ON);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Water heater IoT");
  delay(1000);
};

void loop(){
  // time_t mynext_schedule;

  t_now=now();
  water_temp = get_watertemp();
  time_to_warm = ( target_temp - water_temp) * time_per_degree;		// current time to warm water at this temperature, in secs
  next_schedule = get_next_schedule(schedule);				// Get next time when water has to be warmed, in secs
  Serial.println("Next:");
  print_time(next_schedule);
  Serial.println("Now:");
  print_time(t_now);
  time_to_sched=next_schedule - t_now;
  Serial.print("Time to sched: ");
  Serial.println(time_to_minutes(time_to_sched));
  if (heater_on) running_time = t_now - init_time;		// Calculate running time for displaying
  if (water_temp < target_temp && time_to_warm > time_to_sched && t_now < next_schedule && !stop_programm) {
    //if(debug)Serial.println("Calentando");
    if (!heater_on) {
      power_on_heater();
      init_time = now();
      init_temp = water_temp;
    };
  } else {  // Out of heating period of time
    //if(debug)Serial.println("En espera");
    if (heater_on) {
      power_off_heater();
      final_temp = water_temp;
      if (perform_adjust) time_per_degree = running_time / (final_temp - init_temp); // auto-adjust time per degree
      log_session(); // Send this sesion over RF24
    };
  };
  // if( (count % 2) == 0)
  display(heater_on, water_temp, target_temp, running_time, now(), next_schedule);
  // if( (count % 5) ==0 ) send_data();
  delay(loop_time);
  if(count % 5 == 0)display_show_line();
  count++;
  //if(debug) Serial.println(count);
};

int time_to_minutes(time_t inputtime){
  int return_minutes;
  return_minutes=day(inputtime)*1440 + hour(inputtime)*60 + minute(inputtime);
  return return_minutes;
};

void display_show_line(){
  display_line_i++;
  // Serial.print("Displaying line ");
  // Serial.println(display_line_i);
  if(display_line_i>NUM_DISPLAYS)display_line_i=1;
  delay(50);
};

void display_control_int(){
  display_active++;
  if(display_active>2)display_active=0;
  Serial.println("Display buttons pressed: ");
  Serial.println(display_active);
  switch(display_active){
    case 0:
      lcd.noDisplay();
      ;
    case 1:
      lcd.display();
      lcd.noBacklight();
      ;
    case 2:
      lcd.display();
      lcd.backlight();
      ;
  };
  delay(50);
};

int send_data(){
  radio.stopListening();
  Serial.println("Now sending");
  unsigned long start_time = micros();                             // Take the time, and send it.  This will block until complete
/* Enviar struct */
   if (!radio.write( &start_time, sizeof(unsigned long) )){
     Serial.println("Send failed");
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
      Serial.println("Failed, response ACK timed out.");
  }else{
      unsigned long got_time;                                 // Grab the response, compare, and send to debugging spew
      radio.read( &got_time, sizeof(unsigned long) );
      unsigned long end_time = micros();
      // Spew it
      Serial.print("Sent ");
      Serial.print(start_time);
      Serial.print(", Got response ");
      Serial.print(got_time);
      Serial.print(", Round-trip delay ");
      Serial.print(end_time-start_time);
      Serial.println(" microseconds");
  };
  // Try again 1s later
  delay(3000);
};

void stop_programm_int(){
	// lcd.clear();
    stop_programm=!stop_programm;
    digitalWrite(ProgrammedLed,!stop_programm);
    Serial.println("Stop BUTTON Pressed");
	delay(50);
    // if(LCD_DISPLAY) lcd.clear();
  };

int set_date(time_t date){  // Need to implement an NTP-like using RF24 get_date()
  setTime(23,50,00,1,10 ,2018);
  };

time_t get_date(){ // TODO
  // Gets date and time from external source
  };

int load_schedules(struct dailyprog *schedule){}; // TODO

int show_schedules(struct dailyprog *schedule){
    int sch_i;
    int hour,min;
    for(sch_i=0;sch_i<num_schedules;sch_i++){
      Serial.println("Schedule loop");
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
    Serial.print("Temperature: ");
    Serial.println(sensortemp);
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
    // Serial.print("Searching next schedule: ");
    // Serial.println(sch_i);
      time2=sched[sch_i].weekday*1440 + (sched[sch_i].use_time/12 + 4)*60 + sched[sch_i].use_time%12 * 5;
      // Serial.print("Weekday found: ");
      // Serial.print(time2);
      // Serial.print(" / ");
      // Serial.print(time1);
      // Serial.print(" / ");
      // Serial.println(time2-time1);
      if(time2-time1<time_found && time2-time1>0){  // search for minimum difference (next schedule)
        time_found=time2-time1;
        sch_found=sch_i;
        // Serial.print("Sched found: ");
        // Serial.println(sch_i);
      };
    sch_i++;
  };
  timeElements={0,sched[sch_found].use_time%12 * 5, sched[sch_found].use_time/12 + 4,
                  sched[sch_found].weekday,day(t_now)+(sched[sch_found].weekday-weekday(t_now)),
                  month(t_now),year(t_now)-1970};
  return_time=makeTime(timeElements);
  // Serial.println("Next schedule");
  // print_time(return_time);
  return return_time;
  };

void print_time(time_t sched){
  if(year(sched)>1970){
    Serial.print(day(sched));
    Serial.print("/");
    Serial.print(month(sched));
    Serial.print("/");
    Serial.print(year(sched));
    Serial.print(" ");
  };
  Serial.print(hour(sched));
  Serial.print(":");
  Serial.print(minute(sched));
  Serial.print(":");
  Serial.println(second(sched));
};

int power_on_heater(){ // TODO Implement relay control
  Serial.println("Encendiendo calentador");
  heater_on=1;
  digitalWrite(HeaterPin, HIGH);
  };

int power_off_heater(){ // TODO Implement relay control
  Serial.println("Apagando calentador");
  heater_on=0;
  digitalWrite(HeaterPin, LOW);
  };

int log_session(){}; // TODO

int display(int h_status, float temp1, float temp2, int r_time, time_t today, time_t next){
  char buff1[5], buff2[5];
  // char *buff3="";
  // TODO implement formatting LCD output
  // sprintf(buff3,"Param: %2.2f / %2.2f", temp1, temp2);
  // Serial.println(buff3);
  // Serial.println(buff1);
  // Serial.println(buff2);
  dtostrf(temp1,2,2,buff1);
  dtostrf(temp2,2,2,buff2);
  sprintf(display_line [0], "%02d/%02d    %02d:%02d  ",day(today),month(today),hour(today),minute(today));
  sprintf(display_line [1], "T: %s->%s", buff1, buff2);
  // sprintf(display_line [1], "Temp: %02dC->%02dC  ", temp1, temp2);
  sprintf(display_line [2], "%02d/%02d    %02d:%02d  ",day(next),month(next),hour(next),minute(next));
  // sprintf(display_line [2], "Linea 2         ");
  sprintf(display_line [3], "Linea 3         ");
  if(display_active){ // Display Active
    lcd.setCursor(0,0);
    lcd.print(display_line[0]);
    lcd.setCursor(0,1);
    lcd.print(display_line[display_line_i]);
    // display_show_line();
    // display_control_int();
  }else{ // No display available
    Serial.print("Water temperature: ");
    Serial.println(temp1,DEC);
    Serial.print("Target temperature: ");
    Serial.println(temp2);
    Serial.print("Heater is: ");
    if(h_status)Serial.println("ON"); else Serial.println("OFF");
    Serial.print("Running time: ");
    Serial.println(r_time);
    Serial.print("Date/Time: ");
    Serial.println(today);
    Serial.print("Time to target: ");
    Serial.println(next-today);
  };
    if(debug){
      Serial.print("Water temperature: ");
      Serial.println(temp1,DEC);
      Serial.print("Target temperature: ");
      Serial.println(temp2);
      Serial.print("Heater is: ");
      if(h_status)Serial.println("ON"); else Serial.println("OFF");
      Serial.print("Running time: ");
      Serial.println(r_time);
      Serial.print("Date/Time: ");
      Serial.println(today);
      Serial.print("Time to target: ");
      Serial.println(next-today);
      Serial.print("Performance: ");
      Serial.print(time_per_degree);
      Serial.println(" secs per degree");
      Serial.print("Programmed: ");
      Serial.println(!stop_programm);
      Serial.print("Next: ");
      Serial.println(next);
    };
};

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
// #define LCD_DISPLAY true
// #define RF24 true

// #ifdef RF24
  #include <SPI.h>
  #include "RF24.h"
  #include <LCD.h>
  #include <LiquidCrystal_I2C.h>

  RF24 radio(9,10);
// #endif

#include <OneWire.h>
#include <DallasTemperature.h>

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
int time_per_degree = 5; 			    // time to raise 1 degree, in secs, initial value
int heater_autoadjust = 0;				      // weather time_per_degree is auto adjust from previous data
int max_running_time = 4 * 60;			// 4 mins maximum running time, default = 4 hours
char week[]={"DLMXJVS"};

//Internal variables
int stop_programm = 0;					    // stop schedule running
int heater_on=0;						          // heater status
int sensorValue=0;  // variable to  store  the value  coming  from  the sensor
int count=1;          // Loop count
byte schedule[7];				            // contains usage schedule, 1 hour intervals
float water_temp;			  // current water temperature and target temperature
float init_temp, final_temp;			    // controls autoadjustment
int time_to_warm;	  // time left to reach target_temp, in secs
int time_to_sched, time_in_sched; //
time_t date, next_schedule_ini, next_schedule_fin, t_now;      // date and time. time left for next use
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
  // Prototype
  pinMode(HeaterPin,OUTPUT);
  pinMode(ProgrammedLed, OUTPUT);
  // pinMode(displayPin,INPUT);
  attachInterrupt(digitalPinToInterrupt(displayPin),display_control_int, RISING);
  attachInterrupt(digitalPinToInterrupt(stop_button),stop_programm_int, RISING); // Stop programm interrupt
  attachInterrupt(digitalPinToInterrupt(displayLine),display_show_line,RISING);
  Serial.begin(115200);
  // #ifdef RF24
        radio.begin();
        radio.setPALevel(RF24_PA_LOW);
        radio.openWritingPipe(pipe);
        radio.openReadingPipe(1,pipe);
    // #endif
  ///
  sensorDS18B20.begin();
  date = get_date(); // Ask for remote date
  set_date(date);
  // Prototype. Schedule from 10 seconds to 110 seconds from now.
  next_schedule_ini=now()+10;
  next_schedule_fin=next_schedule_ini+100;
  load_schedule(schedule);
  digitalWrite(ProgrammedLed,!stop_programm);
  // sensors.begin();
// #ifdef LCD_DISPLAY
    lcd.begin(16,2,LCD_5x8DOTS);
    lcd.createChar(0, char_OFF);
    lcd.createChar(1, char_ON);
    lcd.clear();
    // lcd.command(LCD_NOBACKLIGHT );
    // lcd.noBackLight();
    lcd.setCursor(0,0);
    lcd.print("Water heater IoT");
    delay(1000);
// #endif
};

void loop() {
  t_now=now();
  water_temp = get_watertemp();
  time_to_warm = ( target_temp - water_temp) * time_per_degree;		// current time to warm water at this temperature, in secs
  //next_schedule = get_next_schedule();				// Get next time when water has to be warmed, in secs
  time_to_sched=next_schedule_ini - t_now;
  if (heater_on) running_time = t_now - init_time;		// Calculate running time for displaying
  if (water_temp < target_temp && time_to_warm > time_to_sched && t_now < next_schedule_fin && !stop_programm) {
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
  display(heater_on, water_temp, target_temp, running_time, now(), next_schedule_ini);
  // if( (count % 5) ==0 ) send_data();
  delay(loop_time);
  count++;
  //if(debug) Serial.println(count);
};

void display_show_line(){
  display_line_i++;
  Serial.print("Displaying line ");
  Serial.println(display_line_i);
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
  setTime(20,55,00,23,9,2018);
  };

time_t get_date(){ // TODO
  // Gets date and time from external source
  };

int load_schedule(byte *schedule){}; // TODO

float get_watertemp(){  //Implement according to hardware sensor
    float sensortemp;
    // Prototype
    // sensorValue =  analogRead(sensorPin);
    // sensors.requestTemperatures();
    // return sensors.getTempCByIndex(0);
    //return sensorValue;
    sensorDS18B20.requestTemperatures();
    sensortemp=sensorDS18B20.getTempCByIndex(0);
    Serial.println("");
    Serial.println(sensortemp);
    Serial.println("");
    return sensortemp;
    // return 28;
    ///
};

int get_next_schedule(){ // TODO
  time_t return_time;
  //return_time=1516595194;
 // return_time=MakeTime(t_next);
  //return_time=(now()+10000);
  return return_time;
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
  char buff1[5];
  char buff2[5];
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
  sprintf(display_line [2], "Linea 2         ");
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

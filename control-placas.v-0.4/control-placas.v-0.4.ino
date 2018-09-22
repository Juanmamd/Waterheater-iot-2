// TO DO
// check maximum working time.
// serial commands
// several displays screens
// Low power consumption: LCD, RF, Arduino

// Project pinout
// D2	- Stop program button
// D3	- Read temperature
// D3	- Heater LED
// D4	- Programmed LED
// D5	- Not used
// D6	- Not used
// D7	- Not used
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
// #include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <max6675.h>

#define I2C_ADDR    0x3F    // 0x27 in PCF8574 by NXP and Set to 0x3F in PCF8574A
#define LCD_DISPLAY true
#define RF24 true

#ifdef RF24
  #include <SPI.h>
  #include "RF24.h"
  RF24 radio(9,10);
#endif

//User controlled variables
int target_temp=30;

// hardware variables
int sensorPin  =  A0;     // select the input  pin for the potentiometer
int HeaterPin  =  3;   // select the pin for  the LED, relay emulation
const byte stop_button = 2; // Stop programming / manual mode
int ProgrammedLed=4;

//System variables
int debug=1;
int loop_time = 50;  					      // time to sleep earch loop, in secs
int time_per_degree = 5; 			    // time to raise 1 degree, in secs, initial value
int heater_autoadjust = 1;				      // weather time_per_degree is auto adjust from previous data
int max_running_time = 4 * 60;			// 4 mins maximum running time, default = 4 hours
char week[]={"DLMXJVS"};

//Internal variables
volatile int stop_programm = 0;					    // stop schedule running
int heater_on=0;						          // heater status
int sensorValue =  0;  // variable to  store  the value  coming  from  the sensor
int count=1;          // Loop count
byte schedule[7];				            // contains usage schedule, 1 hour intervals
int water_temp, target_temp;			  // current water temperature and target temperature
int init_temp, final_temp;			    // controls autoadjustment
int time_to_warm;	  // time left to reach target_temp, in secs
int time_to_sched, time_in_sched; //
time_t date, next_schedule_ini, next_schedule_fin, t_now;      // date and time. time left for next use
time_t init_time, fin_time, running_time;	// running time control

#ifdef LCD_DISPLAY
  LiquidCrystal_I2C lcd(I2C_ADDR, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
  byte char_ON[] = {  B11111,  B10001,  B10101,  B10101,  B10101,  B10101,  B10001,  B11111  };
  byte char_OFF[] = {  B11111,  B10001,  B10001,  B10001,  B10001,  B10001,  B10001,  B11111  };
  const int NUM_DISPLAYS=2; // Num of rotating screens
  char display_line[2][2][16]; // LCD lines size
  int display_active=1;
  int display_light=1;
#endif

void setup() {
  // Prototype
  pinMode(HeaterPin,OUTPUT);
  pinMode(ProgrammedLed, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(stop_button),stop_programm_int, RISING); // Stop programm interrupt
  Serial.begin(115200);
  #ifdef RF24
        radio.begin();
        radio.setPALevel(RF24_PA_LOW);
        radio.openWritingPipe("ServerIoT");
        radio.openReadingPipe(1,"WaterHeater");
    #endif
  ///
  date = get_date(); // Ask for remote date
  set_date(date);
  // Prototype. Schedule from 10 seconds to 110 seconds from now.
  next_schedule_ini=now()+10;
  next_schedule_fin=next_schedule_ini+100;
  load_schedule(schedule);
  digitalWrite(ProgrammedLed,!stop_programm);
  // sensors.begin();
#ifdef LCD_DISPLAY
  if(LCD_DISPLAY){
    lcd.begin(16,2,LCD_5x8DOTS);
    lcd.createChar(0, char_OFF);
    lcd.createChar(1, char_ON);
    lcd.clear();
    lcd.backlight();
    lcd.setCursor(0,0);
    lcd.print("Water heater IoT");
    delay(1000);
    lcd.clear();
  };
#endif
}

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
      heater_on = 1;
    }
  } else {  // Out of heating period of time
    //if(debug)Serial.println("En espera");
    if (heater_on) {
      if(debug)Serial.println("Apagando calentador");
      heater_on = 0;
      power_off_heater();
      final_temp = water_temp;
      if (perform_adjust) time_per_degree = running_time / (final_temp - init_temp); // auto-adjust time per degree
      log_session(); // Send this sesion over RF24
    }
  }
  if( (count % 2) == 0) display(heater_on, water_temp, target_temp, running_time, now(), next_schedule_ini);
  if( (count % 5) ==0 ) send_data();
  delay(loop_time);
  count++;
  //if(debug) Serial.println(count);
}

int send_data(){
  radio.stopListening();
  Serial.println(F("Now sending"));
  unsigned long start_time = micros();                             // Take the time, and send it.  This will block until complete
/* Enviar struct */
   if (!radio.write( &start_time, sizeof(unsigned long) )){
     Serial.println(F("Send failed"));
   }
  radio.startListening();                                    // Now, continue listening
  unsigned long started_waiting_at = micros();               // Set up a timeout period, get the current microseconds
  boolean timeout = false;                                   // Set up a variable to indicate if a response was received or not
  while ( ! radio.available() ){                             // While nothing is received
    if (micros() - started_waiting_at > 200000 ){            // If waited longer than 200ms, indicate timeout and exit while loop
        timeout = true;
        break;
    }
  }
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
  }
  // Try again 1s later
  delay(3000);
  }
};

void stop_programm_int(){
	// lcd.clear();
    stop_programm=!stop_programm;
    digitalWrite(ProgrammedLed,!stop_programm);
    Serial.println("BUTTON PUSHED");
	delay(50);
    // if(LCD_DISPLAY) lcd.clear();
  };

int set_date(time_t date){  // Need to implement an NTP-like using RF24 get_date()
  setTime(22,00,00,17,9,2018);
  };

time_t get_date(){ // TODO
  // Gets date and time from external source
  };

int load_schedule(byte *schedule){}; // TODO

int get_watertemp(){  //Implement according to hardware sensor
    // Prototype
    // sensorValue =  analogRead(sensorPin);
    // sensors.requestTemperatures();
    // return sensors.getTempCByIndex(0);
    //return sensorValue;
    return 29;
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
  heater_on=1;
  digitalWrite(HeaterPin, HIGH);
  };

int power_off_heater(){ // TODO Implement relay control
  heater_on=0;
  digitalWrite(HeaterPin, LOW);
  };

int log_session(){}; // TODO

int display(int h_status, int temp1, int temp2, int r_time, time_t today, time_t next){
  // TODO implement formatting LCD output
  if(LCD_DISPLAY){ // Display Active

    lcd.setCursor(0,0);
    lcd.print(week[weekday(today)-1]);
    lcd.print("/");
    lcd.print(month(today));
    lcd.print("/");
    lcd.print(day(today));
    lcd.print(" ");
    lcd.print(hour(today));
    lcd.print(":");
    lcd.print(minute(today));
	lcd.setCursor(13,0);
	lcd.print(time_to_sched);
    if(stop_programm){
        lcd.setCursor(0,1);
        lcd.print("Not programmed");
    }else{
      lcd.setCursor(0,1);
	  lcd.write(h_status);
      // if(h_status){
        // lcd.print(" ON");
      // }else{
        // lcd.print("OFF");
      // };
      lcd.setCursor(4,1);
      lcd.print(temp1);
      lcd.print("->");
      lcd.print(temp2);
      lcd.setCursor(13,1);
      lcd.print(r_time);
    };
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

#include <Time.h>
#include <TimeLib.h>

//User controlled variables
int perform_adjust = 1;				      // weather time_per_degree is auto adjust
int max_running_time = 4 * 60;			// 4 hours maximum running time
byte schedule[7];	
int target_temp=30;			            // contains usage schedule, 1 hour intervals

//System variables
// Prototype
int debug=1;
int sensorPin  =  A0;     // select the input  pin for the potentiometer 
int HeaterPin  =  12;   // select the pin for  the LED
int sensorValue =  0;  // variable to  store  the value  coming  from  the sensor
int ProgrammedLed=13;
const byte stop_button = 2; // Stop programming / manual mode
int time_per_degree = 5; 			    // time to raise 1 degree, in secs
int loop_time = 100;  					      // time to sleep earch loop, in secs
int heater_on=0;						          // heater status
volatile int stop_programm = 0;					    // stop schedule running

//Internal variables
int count=1;
int time_to_warm;	  // time left to reach target_temp, in secs
int time_to_sched, time_in_sched;
time_t date, next_schedule_ini, next_schedule_fin, t_now;						            // date and time. time left for next use
time_t init_time, fin_time, running_time;	// running time control
int water_temp;			  // current water temperature vs target temperature
int init_temp, final_temp;			    // controls autoadjustment

//Prototype
#include <OneWire.h>
#include <DallasTemperature.h>
// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 3
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
//

void stop_programm_int(){
    stop_programm=!stop_programm;
    digitalWrite(ProgrammedLed,!stop_programm);
    Serial.println("BUTTON PUSHED");
  };

int set_date(time_t date){
  setTime(22,45,00,21,1,2018);
  };
time_t get_date(){
  // Gets date and time from external source
  };

int load_schedule(byte *schedule){
	
	
};

int get_watertemp(){
    // Prototype
    sensorValue =  analogRead(sensorPin);
    sensors.requestTemperatures();
    return sensors.getTempCByIndex(0);
    //return sensorValue;
    ///
};
int get_next_schedule(){
  time_t return_time;
  //return_time=1516595194;
 // return_time=MakeTime(t_next);
  //return_time=(now()+10000);
  return return_time;
  };
  
int power_on_heater(){
  heater_on=1;
  digitalWrite(HeaterPin, HIGH);
  };
  
int power_off_heater(){
  heater_on=0;
  digitalWrite(HeaterPin, LOW);
  };
  
int log_session(){
	
};

int display(int h_status, int temp1, int temp2, int r_time, time_t today, time_t next){
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
  if(debug){
    Serial.print("Performance: ");
    Serial.print(time_per_degree);
    Serial.println(" secs per degree");
    Serial.print("Programmed: ");
    Serial.println(!stop_programm);
    Serial.print("Next: ");
    Serial.println(next);
  };
};

void setup() {
  // Prototype
  pinMode(HeaterPin,OUTPUT);
  pinMode(ProgrammedLed, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(stop_button),stop_programm_int, RISING);
  Serial.begin(9600);
  ///
  date = get_date();
  set_date(date);
  // Prototype
  next_schedule_ini=now()+100;
  next_schedule_fin=next_schedule_ini+100;
  /// 
  load_schedule(schedule);
  digitalWrite(ProgrammedLed,!stop_programm);
  sensors.begin();
}

void loop() {
  t_now=now();
  water_temp = get_watertemp();
  time_to_warm = ( target_temp - water_temp) * time_per_degree;		// current time to warm water at this temperature, in secs
  //next_schedule = get_next_schedule();				// Get next time when water has to be warmed, in secs
  time_to_sched=next_schedule_ini - t_now;
  if (heater_on) running_time = t_now - init_time;		// Calculate running time for displaying
  if (water_temp < target_temp && time_to_warm > time_to_sched && t_now < next_schedule_fin && !stop_programm) {
    if(debug)Serial.println("Calentando");
    if (!heater_on) {
      power_on_heater();
      init_time = now();
      init_temp = water_temp;
      heater_on = 1;      
    }
  } else {
    //if(debug)Serial.println("En espera");
    if (heater_on) {
      if(debug)Serial.println("Apagando calentador");
      power_off_heater();
      heater_on = 0;
      final_temp = water_temp;
      if (perform_adjust) time_per_degree = running_time / (final_temp - init_temp); // auto-adjust time per degree
      log_session();
    }
  }
  if( (count % 5) == 0) display(heater_on, water_temp, target_temp, running_time, now(), next_schedule_ini); // Refresh display every 5 loops
  delay(loop_time);
  count++;
  //if(debug) Serial.println(count);
}


// TO DO
// check maximum working time.

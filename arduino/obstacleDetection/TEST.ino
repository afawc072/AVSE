//NewPing Library for Arduino
//Author:  Tim Eckel
//Contact: tim@leethost.com
#include <NewPing.h>

#define SONAR_NUM     5 // Number of sensors.
#define CYCLE         3 // Number of cycle
#define MAX_DISTANCE 50 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 50 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define MIN_DISTANCE 15

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentCycle=0;             
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(2, 3, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(4, 5, MAX_DISTANCE),
  NewPing(6, 7, MAX_DISTANCE),
  NewPing(8, 9, MAX_DISTANCE),
  NewPing(10, 11, MAX_DISTANCE),
};

void setup() {
  Serial.begin(9600);
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
}

void loop() {
    bool feedback = updateSensors();

}


bool updateSensors(){
    bool noWarning = true;
    // Get measures
    for (currentSensor = 0; currentSensor < SONAR_NUM; currentSensor++) {                           // Loop through all the sensors.
        float echoTime = sonar[currentSensor].ping_median(CYCLE); 
        cm[currentSensor] = sonar[currentSensor].convert_cm(echoTime);                     // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
   
        if(cm[currentSensor] != 0 && cm[currentSensor] < MIN_DISTANCE)
        {
           cm[currentSensor] = -1;
           noWarning = false;
        }
      }
    Serial.print(noWarning);
  Serial.print(" Distances: ");
  for(int i = 0; i < SONAR_NUM ; i++){
    Serial.print(cm[i]);
    Serial.print("                  ");
  }
  Serial.println();
      
     return noWarning;
}

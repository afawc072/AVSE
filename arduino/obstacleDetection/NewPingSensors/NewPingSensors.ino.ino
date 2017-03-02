#include <NewPing.h>

#define SONAR_NUM     2 // Number of sensors.
#define CYCLE         2 // Number of cycle
#define MAX_DISTANCE 50 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 50 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[CYCLE][SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentCycle=0;
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

uint8_t final_distances[SONAR_NUM];

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(2, 3, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(4, 5, MAX_DISTANCE),
//  NewPing(6, 7, MAX_DISTANCE),
//  NewPing(8, 9, MAX_DISTANCE),
//  NewPing(10, 11, MAX_DISTANCE),
};

void setup() {
  Serial.begin(9600);
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
}

void loop() {
   for (uint8_t j = 0; j < CYCLE; j++) {                                    // Two cycle loop
    for (uint8_t i = 0; i < SONAR_NUM; i++) {                           // Loop through all the sensors.
      if (millis() >= pingTimer[i]) {                                   // Is it this sensor's time to ping?
        pingTimer[i] += PING_INTERVAL * SONAR_NUM;                      // Set next time this sensor will be pinged.
        if (i == 0 && currentSensor == SONAR_NUM - 1) threeSensorCycle(); // Sensor ping cycle complete, do something with the results.
        sonar[currentSensor].timer_stop();                              // Make sure previous timer is canceled before starting a new ping (insurance).
        currentSensor = i;                                              // Sensor being accessed.
        currentCycle = j;                                               // Cycle
        cm[currentCycle][currentSensor] = 0;                            // Make distance zero in case there's no ping echo for this sensor.
        sonar[currentSensor].ping_timer(echoCheck);                     // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
      }
    }
  }
}


void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentCycle][currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void threeSensorCycle() {
  // Setting threshold to print useful data and send to the raspberry pi
  for (uint8_t j = 0; j < CYCLE; j++) {
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
          if (cm[j][i] >= 20) {     // The object is inside the specify range and send the info to the raspberry
              if(i==0)checkValueSensor1(i);
              if(i==1)checkValueSensor2();
                  }
           else if (cm[j][i] < 20 && cm[j][i] >0) {
//             The objet is close to the robot !!!!!
//             Send something to warn a collision
                Serial.print(i); 
                Serial.print(j);
                Serial.print("=");
                Serial.print("X");
                Serial.print("cm ");
          }   
          else {
                Serial.print(i); 
                Serial.print(j);
                Serial.print("=");
                Serial.print(cm[j][i]);
                Serial.print("cm ");
          }
        }
 Serial.println();
}
}

void checkValueSensor1(int i){     // Check if the difference between value is between 0 and 3 --> ERROR CHECK
  int checkOne=abs(cm[0][i]-cm[1][i]);
  int sendSensorOne=(cm[0][i]+cm[1][i])/2;
    if(checkOne>=0 && checkOne<3){
      Serial.print("Sensor 1"); 
      Serial.print("=");
      Serial.print(sendSensorOne);
      final_distances[SONAR_NUM]=sendSensorOne;
}
}

void checkValueSensor2(){     // Check if the difference between value is between 0 and 3 --> ERROR CHECK
  int checkTwo=abs(cm[0][1]-cm[1][1]);
  int sendSensorTwo=(cm[0][1]+cm[1][1])/2;
    if(checkTwo>=0 && checkTwo<3){
      Serial.print("Sensor 2"); 
      Serial.print("=");
      Serial.print(sendSensorTwo);
}
}


//void loop() {
//  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
//    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
//      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
//      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
//      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
//      currentSensor = i;                          // Sensor being accessed.
//      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
//      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
//    }
//  }
//}
//void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
//  for (uint8_t i = 0; i < SONAR_NUM; i++) {
//    Serial.print(i);
//    Serial.print("=");
//    Serial.print(cm[i]);
//    Serial.print("cm ");
//  }
//  Serial.println();
//}

// Original code
//void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
//  // The following code would be replaced with your code that does something with the ping results.
//  for (uint8_t i = 0; i < SONAR_NUM; i++) {
//    Serial.print(i);
//    Serial.print("=");
//    Serial.print(cm[i]);
//    Serial.print("cm ");
//  }
//  Serial.println();
//}

//void oneSensorCycle() {
//  // Setting threshold to print useful data and send to the raspberry pi
//  for (uint8_t i = 0; i < SONAR_NUM; i++) {
//          if (cm[i] >= 20) {     // The object is inside the specify range and send the info to the raspberry
//              Serial.print(cm[i]);
//              Serial.print("cm......");
//          }          else if (cm[i] < 20 && cm[i] >0) {
//            // The objet is close to the robot !!!!!
//            // Send something to warn a collision
//            Serial.print("X.........");
//          }   
//          else {
//            Serial.print("0.........");
//          }
//        }
// Serial.println();
//}



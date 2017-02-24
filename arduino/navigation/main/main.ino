#include <Servo.h>
#include <math.h> 

//ODOMETRY VARIABLES
int leftCount = 0;  
int leftCountOld = 0;
int leftCountNew = 0; //Total left ticks
int rightCount = 0; 
int rightCountOld = 0;
int rightCountNew = 0; //Total right ticks

unsigned long timeOld = 0; //Time variable
unsigned long timeCurrent;
unsigned long dt; //Time difference

byte irLeft; //Value of left wheel encoder
byte irRight; //Value of right wheel encoder

//CONSTANTS
const float pi = 3.1415; //Pi
const float wheelTicks = 64; //Total ticks per wheel rotation
const float wheelRadius = 2.7; //In cm
const float wheelBase = 19.05; //In cm

//Translational Velocity
double Vo = 0;

//LOCATION VARIABLES
float distanceLeft = 0; //Distance travelled by left wheel
float distanceRight = 0; //Distance travelled by right wheel
float distanceCenter = 0; //Distance travelled by center of robot

float oldDistanceLeft = 0;
float oldDistanceRight = 0;
float oldDistanceCenter = 0;
float deltaDistanceLeft = 0;
float deltaDistanceRight = 0;
float deltaDistanceCenter = 0;

float positionX = 0; //X position of the robot
float positionXOld = 0;
float positionY = 0; //Y position of the robot
float positionYOld = 0;
float heading = 0; //Heading angle of the robot
float headingOld = 0;

//PID
float dX; //X difference
float dY; //Y difference

float w; //Angular Velocity


//float finalX; //Final X position of the destination
//float finalY; //Final Y position of the destination
float goalHeading; //Temporary (or final?) goal heading of position to reach

float errorP=0; //P error for the PID
float errorI=0; //I error for the PID
float errorD=0; //D error for the PID
float errorHeading=0;
float errorPrevious=0; //Extra error for PID calculations

float Kp = 50; //P tuning parameter
float Ki = 1/1000; //I tuning parameter
float Kd = 0.1; //D tuning parameter

float velocityLeft;
float velocityRight;
float mappedVelocityLeft;
float mappedVelocityRight;

boolean moving = false; //To be used with navigation system

int tempCount = 0;

Servo leftServo; //Servo motor to control the left wheel
Servo rightServo; //Servo motor to control the right wheel
Servo cameraServo; //Servo motor to control the camera

unsigned long t1 = 0;
unsigned long t2 = 0;

int test = 2000;

// Yannick
const int NUM_SENSORS = 5;
float DIST_THRESHOLD_CM = 50;
float obstacle_distances[NUM_SENSORS] = {100,100,100,100,100};
float bufferzone_distances[NUM_SENSORS] = {0,0,0,0,0};

// Protocol
char MSG_START = '[';
char MSG_SPACE = ':';
char MSG_END = ']';

/*******************************************************************
* Setup function
 *******************************************************************/
void setup() {
  
    Serial.begin(9600);
    pinMode(2,INPUT); //irLeft input pin
    pinMode(3,INPUT); //irRight input pin
    
    t1 = millis();
    
    leftServo.attach(36); //Attach left wheel servo
    rightServo.attach(48); //Attach right wheel servo
    cameraServo.attach(9); //Attach camera servo

    //Setup interrupts for encoders, to be called on change
    attachInterrupt(0, updateLeftTick, CHANGE);
    attachInterrupt(1, updateRightTick, CHANGE);

    leftServo.writeMicroseconds(1355);
    rightServo.writeMicroseconds(1330);
    /*
    //CALL FUNCTION FOR SWEEP TO FIND TAG
    //findFirstTag();

    //GET POSITION FROM RASPBERRY PI
    //getInitialPosition();
    //moving = true;
    //TEMPORARY VALUES USED FOR TESTING
    goalX = 0; //X value for testing
    goalY = 0; //Y value for testing
    

    moving = true; //To be changed after integration with Pi to be modified after we get the intial position.

    */
}



/*******************************************************************
* Main loop
*******************************************************************/
void loop() {
  
  // Verify if serial connection is available    
  if (Serial.available() > 0)
  {
    Serial.flush();

    // Verify if header is present in the message
    String header= Serial.readStringUntil(MSG_START);
    if(header.length() != 0) 
    {  
      sendToPi("ERROR","MISSING_HEADER");
    }
    else
    {
      // Read command
      String cmd = Serial.readStringUntil(MSG_SPACE);
      cmd.trim();

      // Read information
      String info = Serial.readStringUntil(MSG_END);
      info.trim();

      String dump = Serial.readString(); //We ignore the data after the ] symbol

      // ANALYZE COMMAND

      // The RPi tests the connection
      if(cmd.equals("READY"))
      {
        sendToPi("LISTEN","");
      }

      // The RPi wants to send the next position vector (in respect to robot's orientation)
      else if(cmd.equals("NEXT"))
      { 
        // Get the x and y distances       
        float x, y;
        char* y_char = strchr(info.c_str(),',');
        x = atof(info.c_str());
        y = atof(++y_char);

        bool reached = false;
        
        reached = goToGoal(y,-x);
        //test_LED(round(x+y)); //Temporary test to see if vector was properly received
        
        if(reached)
        {  
           // YANNICK FUNCTION -> format : updateSensorDistances();
           updateBufferzoneDist();
          
           String infoSensor = "";
           int i;      
           // Loop to create the info section of the message containing the distances in the buffer zone (0 if not)    
           while( i < (NUM_SENSORS-1) )
           {
               infoSensor += String(bufferzone_distances[i++])+",";
            }
            infoSensor += String(bufferzone_distances[i]);
            
            sendToPi("REACHED",infoSensor);   
        }
        else
        {
           sendToPi("ERROR","UNREACHED"); 
         }
        
      }
      
      // The RPi wants to set the camera sevo angle
      else if(cmd.equals("CAMANGLE"))
      {
         float angle = atof(info.c_str());
  
         test_LED(angle); // temporary test to see if angle was properly received
        
         bool flagCam = true;    
           
         // CAM ANGLE FUNCTION -> format : flagCam = setCamAngle(angle)
         if(flagCam)
         {
            sendToPi("END","");
          }
         else
         {
            sendToPi("ERROR","SETCAMERROR");
          }
       }
      else
      {
         sendToPi("ERROR","UNKNOWN_CMD");
       }
    }  
  }
  
  else
  {
   // Ignore -> serial is not available
  }
}


/*******************************************************************
* Format the command and information and send it to the Raspberry Pi 
 *******************************************************************/
void sendToPi(String command, String info)
{
  String message = MSG_START + command +MSG_SPACE+info+MSG_END;
  Serial.print(message);
}


/*******************************************************************
* Function that loops through the obstacle_distances array and sends
* a message to the RPi to add the obstacles in the grid.
*******************************************************************/
void updateBufferzoneDist(){
    /*
    * The goal here would be to loop through sensor distances
    * to determine if a distance is smaller than threshold.
    * 
    * ---> WILL IT BE POSSIBLE TO DETECT AN OBSTACLE WITH 1 OR 3 SENSORS? 
    * ---> IF ONLY 1 DETECTS A SMALL DISTANCE, HOW TO ADD OBSTACLE? (W/ 2TRIANGULATION OK)
    
    for(int i=0 ; i < NUM_SENSORS ; i++)
    {
      if(condition to be in bufferzone - using MAX_DIST and MIN_DIST ?)
      {
         bufferzone_distances[i] = obstacle_distances[i];
      }
      else
      {
         bufferzone_distances[i] = 0;
      }
    }
    */
}

/*******************************************************************
* Function to bring the robot from its current position
* to the next one in the occupency grid
*******************************************************************/
bool goToGoal(float dX, float dY)
{ 
  
  //Need to reset positionX, positionY, distanceLeft, distanceRight, distanceCenter and heading to 0???

  //***********************************************************

  //Reset robot position back to (0,0)
  positionX = 0;
  positionY = 0;
  //heading = 0;

  //Reset odometry
  leftCountNew = 0; 
  rightCountNew = 0;
  distanceLeft = 0;
  oldDistanceLeft = 0;
  oldDistanceRight = 0;
  oldDistanceCenter = 0;
  distanceRight = 0;
  distanceCenter = 0;
  
  //************************************************************
  
  bool flagReach = false;
  while(!flagReach)
  {
    getPosition(); //Get position from encoders
   
    
    goalHeading = atan(dY / dX); //Heading required to reach the goal
    errorHeading = goalHeading - heading; //Error with current heading
    errorHeading = atan2(sin(errorHeading),cos(errorHeading)); 
    
    //With time
    timeCurrent = micros();
  
    dt = timeCurrent - timeOld; //Time difference since last execution
    dt = dt / 1000; //Convert to seconds
  
    float X2 = pow(dX, 2); //dX^2
    float Y2 = pow(dY, 2); //dY^2
    Vo = 2 * sqrt(Y2 + X2); //Translational velocity, currently bassed on distance to goal
  
    
    errorP = errorHeading; //Proportional error
    errorI = errorI + (errorHeading * dt); //Integral error
    errorD = (errorHeading - errorPrevious) / dt; //Derivative error

    Serial.print("eH: ");
    Serial.print(errorHeading);
    Serial.print(" pX: ");
    Serial.print(positionX);
    Serial.print(" pY: ");
    Serial.println(positionY);
    
    errorPrevious = errorHeading; //Store previous error
    timeOld = timeCurrent; //Reset time
    
    w = (Kp * errorP) + (Ki * errorI) + (Kd * errorD); //Result of PID
    
    velocityLeft = ((2*Vo - w*wheelBase) / (2*wheelRadius)); //Calculated velocity of left wheel
    velocityRight = ((2*Vo + w*wheelBase) / (2*wheelRadius)); //Calculated velocity of right wheel
    
    velocityToPWM();
    //FUNCTION FOR REACHED 
    
    flagReach = destinationReached(dX, dY);
  }

  leftServo.writeMicroseconds(1355);
  rightServo.writeMicroseconds(1330);

  return flagReach;
  
}

/*******************************************************************
* Function to get the current position (x,y) and heading of the
* robot based on the values obtained from the odometry
*******************************************************************/
void getPosition()
{
  distanceLeft = 2 * pi * wheelRadius * (leftCountNew / wheelTicks); //Distance traveled by the left wheel
  distanceRight = 2 * pi * wheelRadius * (rightCountNew / wheelTicks); //Distance traveled by the right wheel
  distanceCenter = (distanceLeft + distanceRight) / 2; //Distance traveled by the center of the robot

  //Change since last call
  deltaDistanceLeft = distanceLeft - oldDistanceLeft;
  deltaDistanceRight = distanceRight - oldDistanceRight;
  deltaDistanceCenter = distanceCenter - oldDistanceCenter;
  
  positionX = positionX + deltaDistanceCenter * cos(heading); //X position of the robot
  positionY = positionY + deltaDistanceCenter * sin(heading); //Y position of the robot
  heading = heading + (deltaDistanceRight - deltaDistanceLeft) / wheelBase; //Current heading of the robot

  //Reseting the values for the next call
  oldDistanceLeft = distanceLeft;
  oldDistanceRight = distanceRight;
  oldDistanceCenter = distanceCenter;
}


/*******************************************************************
* Function to get convert the output velocities of the PID
* into usable values by the servo motors
*******************************************************************/
void velocityToPWM()
{
  //leftServo zero point = 1355ms
  //rightServo zero point = 1330ms
  
  //Make sure the values are in a range of 180
  if (velocityLeft >=90)
  {
    velocityLeft = 90;
  }
  else if (velocityLeft <=-90)
  {
    velocityLeft = -90;
  }
  if (velocityRight >=90)
  {
    velocityRight = 90;
  }
  else if (velocityRight <=-90)
  {
    velocityRight = -90;
  }

  //Make velocities between 0 and 180
  velocityLeft = velocityLeft + 90;
  velocityRight = velocityRight + 90;

  //Translate value to some usable by the writeMicroseconds() function
  //Range between value is kept the same around their zero point
  //Right wheel input for the map() function has been inverted because the motor is mounted the opposite way on the robot
  mappedVelocityLeft = map(velocityLeft, 0, 180, 1025, 1685); //Keep same range as other wheel
  mappedVelocityRight = map(velocityRight, 180, 0, 1000, 1660);
    
  //If the wheel is suppose to go in reverse, make it stop as to not mess with the odometry ticks
  if(mappedVelocityLeft <= 1355)
  {
    mappedVelocityLeft = 1355;
  }
  else if (mappedVelocityRight >=1330)
  {
    mappedVelocityRight = 1330;
  }

  //Write the speed values into the motors to make them rotate appropriatly
  leftServo.writeMicroseconds(mappedVelocityLeft);
  rightServo.writeMicroseconds(mappedVelocityRight);
}


/*******************************************************************
* Function to update the left wheel tick count on interrupt
*******************************************************************/
void updateLeftTick()
{
  leftCountNew++;
}

/*******************************************************************
* Function to update the right wheel tick count on interrupt
*******************************************************************/
void updateRightTick()
{
  rightCountNew++;
}

/*******************************************************************
* Function to verify if the current destination has been reached
*******************************************************************/
bool destinationReached(float goalX, float goalY)
{
  //Verify if goal is reached, return true if yes, false if not
  if(round(abs(positionX)) > abs(goalX) && round(abs(positionY)) > abs(goalY))
  {
    return true;
  }
  else
  {
    return false;
  }
}


/*******************************************************************
* Temporary test function
 *******************************************************************/
void test_LED(int n){
  for(int i = 0 ; i<n; i++){
      digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(500);                       // wait for a second
      digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
      delay(500); 
    }
}

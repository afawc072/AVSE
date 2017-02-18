/*
CURRENT BUG: Robot has trouble with straight lines or anyhing that involves going in the negative Y axis

TODO:
      IMPLEMENT SWEEP CAMERA SERVO AT START
      IMPLEMENT COMMUNICATION WITH RASPBRRY PI
      
*/

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

float goalX; //Temporary goal X position to reach
float goalY; //Temporary goal Y position to reach
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

void setup() 
{
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
    
    //CALL FUNCTION FOR SWEEP TO FIND TAG
    //findFirstTag();

    //GET POSITION FROM RASPBERRY PI
    //getInitialPosition();
    //moving = true;
    //TEMPORARY VALUES USED FOR TESTING
    goalX = 50; //X value for testing
    goalY = 50; //Y value for testing
    

    moving = true; //To be changed after integration with Pi to be modified after we get the intial position.
}

//Main loop, to be run at all time
void loop() 
{  
    //Directions are inverted on physical robot hence the change in the value
    
      t2 = millis(); //Get time at start of loop

      //wait 0.1 sec for to let tick count update
      if(t2-t1 >= 100)
      {
        t1 = t2;
        setupGoToGoal(); //Update position and error to goal
      }
      else
      {
        getPosition();  //Update position
      }
}

void setupGoToGoal()
{ 
  goToGoal();
  velocityToPWM();
}



void goToGoal()
{ 
  getPosition(); //Get position from encoders
  
  dX = goalX - positionX; //Difference in X position
  dY = goalY - positionY; //Difference in Y position
  
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
  
  errorPrevious = errorHeading; //Store previous error
  timeOld = timeCurrent; //Reset time
  
  w = (Kp * errorP) + (Ki * errorI) + (Kd * errorD); //Result of PID
  
  velocityLeft = ((2*Vo - w*wheelBase) / (2*wheelRadius)); //Calculated velocity of left wheel
  velocityRight = ((2*Vo + w*wheelBase) / (2*wheelRadius)); //Calculated velocity of right wheel
  
}

//Get the current x,y position and heading of the robot
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

//Convert velocity given by PID to value usable by the servos
//leftServo zero point = 1355ms
//rightServo zero point = 1330ms
void velocityToPWM()
{
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


//Function to update the tick count of left wheel on interrupt
void updateLeftTick()
{
  leftCountNew++;
}

//Function to update the wheel count of right wheel on interrupt
void updateRightTick()
{
  rightCountNew++;
}

//void destinationReached()
//{
//    if ((positionX == goalX) && (positionY == goalY))
//    {
//      stopMotor();
//    }
//}
//


/*
 * TO BE IMPLEMENTED
void getInitialPosition()
{
  //Use protocol go get initial position from the raspberry pi
}

void findFirstTag()
{
  //Either sweep servo motor until stop command from pi and send camera servo angle
  //OR
  //Get specific angle from pi to move servo to that angle
}

*/

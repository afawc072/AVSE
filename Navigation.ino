/*
TODO: VERIFY GO TO GOAL FUNCTION
      FINISH ACIVATE MOTORS FUNCTION
      IMPLEMENT SWEEP CAMERA SERVO AT START
      IMPLEMENT INITIALLOCATION FUNCTION
      TEST PROGRAM
      IMPLEMENT COMMUNICATION WITH RASPBRRY PI
*/

#include <Servo.h>

//ODOMETRY VARIABLES
int totalLeftTicks = 0; //Counting total ticks from left wheel 
int totalRightTicks = 0; //Counting total ticks from right wheel
int leftCount = 0;
int leftCountOld = 0;
//int leftCountNew = 0;
int rightCount = 0;
int rightCountOld = 0;
//int rightCountNew = 0;

unsigned long initTime; //Time variable

byte irLeft; //Value of left wheel encoder
byte irRight; //Value of right wheel encoder

//CONSTANTS
const float pi = 3.1415; //Pi
const float wheelTicks = 64; //Total ticks per wheel rotation
const float wheelRadius = 2.125; //DOUBLE CHECK, in cm
const float wheelBase = 21; //DOUBLE CHECK, in cm

//LOCATION VARIABLES
float distanceLeft = 0; //Distance travelled by left wheel
float distanceRight = 0; //Distance travelled by right wheel
float distanceCenter = 0; //Distance travelled by center of robot

float positionX = 0; //X position of the robot
float positionXOld = 0;
float positionY = 0; //Y position of the robot
float positionYOld = 0;
float heading = 0; //Heading angle of the robot
float headingOld = 0;

//PID
float dX; //X difference
float dY; //Y difference

float w;

float goalX; //Temporary (or final?) goal X position to reach
float goalY; //Temporary (or final?) goal Y position to reach
float goalHeading; //Temporary (or final?) goal heading of position to reach

float errorP=0; //P error for the PID
float errorI=0; //I error for the PID
float errorD=0; //D error for the PID
float errorHeading=0;
float error2=0; //Extra error for PID calculations

float P=0; 
float I=0;
float D=0;

float Kp=12; //P tuning parameter
float Ki = 1/1000; //I tuning parameter
float Kd=4; //D tuning parameter

boolean moving = false;

Servo leftServo; //Servo motor to control the left wheel
Servo rightServo; //Servo motor to control the right wheel
Servo cameraServo; //Servo motor to control the camera

void setup() 
{
    Serial.begin(9600);
    pinMode(52,INPUT); //irLeft input pin
    pinMode(42,INPUT); //irRight input pin
    
    initTime = micros(); //Set initial time
    
    irLeft = digitalRead(52); //Read initial value of left ir
    irRight = digitalRead(42); //Read initial value of right ir
    
    leftServo.attach(2); //Attach left wheel servo
    rightServo.attach(48); //Attach right wheel servo
    cameraServo.attach(9); //Attach camera servo
    
    //CALL FUNCTION FOR SWEEP TO FIND TAG
    //findFirstTag();

    //GET POSITION FROM RASPBERRY PI
    //getInitialPosition();
    //moving = true;
    //TEMPORARY VALUES USED FOR TESTING
    goalX = 100; //X value for testing
    goalY= 100; //Y value for testing
}

void loop() 
{
    //Directions are inverted on physical robot hence the change in the value
    //leftServo.writeMicroseconds(2000); //Set left wheel to full speed forward
    //rightServo.writeMicroseconds(1000); //Set right wheel to full speed forward
    
    updateLeftTick(); //Get change in left wheel encoder
    updateRightTick(); //Get change in right wheel encoder


    goToGoal(); //Call the go to goal funcion
    checkV(); //Make sure velocity is whitin he limits of values for the motor
    activateMotors(); //Send velocity to the motors
    //DONT FORGET TO REVERSE DIRECTION OF RIGHT MOTOR!!!!!
}

void goToGoal()
{
  //If we have moved, get position from encoders, if position is already given by first tag location
  if (moving = true)
  {
    getPosition(); //Get the current position of the robot from the odometry
  }
  else
  {
    dX = goalX - positionX; //X difference
    dY = goalY - positionY; //Y difference
    goalHeading = atan2(dX,dY); //Heading of goal
    
    errorHeading = goalHeading - heading; //Error of heading
    errorHeading = atan2(sin(errorHeading),cos(errorHeading)); 

    errorP = errorHeading; 
    P = Kp * errorP;
    errorI = errorP + errorHeading;
    I = Ki * errorI;
    errorD = errorHeading - error2;
    D = Kd * errorD;

    error2 = errorHeading;
    
    //DOUBLE CHECK FOLLOWING LINE
    w = P+I+D; //? not headin or errorHeading??
  }
}

void getPosition()
{
    leftCount =  totalLeftTicks - leftCountOld; //Left ticks since last update
    rightCount = totalRightTicks - rightCountOld; //Righ ticks since last update
    
    distanceLeft = 2 * pi * wheelRadius * (totalLeftTicks / wheelTicks); //Distance travelled by left wheel since last update
    distanceRight = 2 * pi * wheelRadius * (totalLeftTicks / wheelTicks); //Distance travelled by rih wheel since last update
    distanceCenter = (distanceLeft + distanceRight) / 2; //Distance tavelld by center of robot since last update

    positionX = positionXOld * cos(heading); //Update X position
    positionY = positionYOld * sin(heading); //Update Y position
    heading = headingOld + ((distanceLeft - distanceRight) / wheelBase); //Update heading
    heading = atan2(sin(heading),cos(heading));
    
    //Replace old values with the new ones
    leftCountOld = totalLeftTicks; 
    rightCountOld = totalRightTicks;
    positionXOld = positionX;
    positionYOld = positionY;
    headingOld = heading;
    
    velociyLeft = w * (wheelRadius + (wheelBase / 2)); //Planned velociy of left wheel
    velocityRight = w * (wheelRadius - (wheelBase / 2)); //Planned velocity of right wheel
    
}

//Function to make sure that the velociy is whitin the bounds of the motor speed
void checkV()
{
  if (velociyLeft > 255) //Is velocity above maximum orward speed?
  {
    velocityLeft = 255; //Set speed as maximum forward speed
  }
  else if (velocityLeft < -255) //Is velocity "below" maximum reverse speed?
  {
    velociyLeft = -255; //Set speed as maximum reverse speed
  }
  
  if (velocityRight > 255) //Is velociy below maximum forward speed?
  {
    velocityRight = 255;  //Set speed as maximum foward speed
  }
  else if (velocityRight < -255) //Is velocity "below" maximum reverse speed?
  {
    velocityRigth = -255; //Set speed as maximum reverse speed
  }
  
}

//Function to update the tick count of left wheel
void updateLeftTick()
{
    byte irLeftTemp = digitalRead(52); //Read encoder
    
    if (irLeftTemp != irLeft) //Is value read different from the previous one?
    {
      totalLeftTicks++;      //If yes, increase tick count
      irLeft = irLeftTemp; //Update current value of encoder
    }
}

//Function to update the wheel count of right wheel
void updateRightTick()
{
    byte irRightTemp = digitalRead(42); //Read encoder
 
    if (irRightTemp != irRight) //Is value different from the previous one?
    {
        totalRightTicks++; //If yes, increase tick count
        irRight = irRightTemp; //Update current value of encoder
    }
}

/*
void getInitialPosition()
{

}

void findFirstTag()
{

}



/*
TODO:
      IMPLEMENT SWEEP CAMERA SERVO AT START
      TEST PROGRAM
      IMPLEMENT COMMUNICATION WITH RASPBRRY PI

      PID potential info
      https://w3.cs.jmu.edu/spragunr/CS354/handouts/pid.pdf
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
const float wheelRadius = 2.125; //DOUBLE CHECK, in cm
const float wheelBase = 21; //DOUBLE CHECK, in cm

//For wheel velocity equations
double Vo = 0;

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

float Kp = 10; //P tuning parameter
float Ki = 1/10000; //I tuning parameter
float Kd = 0.1; //D tuning parameter

float velocityLeft;
float velocityRight;

boolean moving = false;

Servo leftServo; //Servo motor to control the left wheel
Servo rightServo; //Servo motor to control the right wheel
Servo cameraServo; //Servo motor to control the camera

unsigned long t1 = 0;
unsigned long t2 = 0;

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
    attachInterrupt(0, updateRightTick, CHANGE);
    attachInterrupt(1, updateLeftTick, CHANGE);
    
    //CALL FUNCTION FOR SWEEP TO FIND TAG
    //findFirstTag();

    //GET POSITION FROM RASPBERRY PI
    //getInitialPosition();
    //moving = true;
    //TEMPORARY VALUES USED FOR TESTING
    goalX = 10; //X value for testing
    goalY= 30; //Y value for testing
    

    moving = true; //To be changed after integration with Pi to be modified after we get the intial position.
}

void loop() 
{
    //Directions are inverted on physical robot hence the change in the value
    
    //timeBefore = millis(); //Get time at start of loop

   
//    goToGoal(); //Call the go to goal funcion
//    convertVelocityToMS(); //Call to convert velocity to a value usable by servo motors
//
//    destinationReached();

      t2 = millis();

      //wait 0.1 sec
      if(t2-t1 >= 100)
      {
        t1 = t2;
        setupGoToGoal();
        
    Serial.print("pX: ");
    Serial.print(positionX);
    Serial.print(", pY: ");
    Serial.print(positionY);
    Serial.print(", heading: ");
    Serial.print(heading);
    Serial.print(", w:");
    Serial.println(w); 

//    Serial.print("errorP: ");
//    Serial.print(errorP);
//    Serial.print(", errorI: ");
//    Serial.print(errorI);
//    Serial.print(", errorD: ");
//    Serial.print(errorD);
//    Serial.print(", w:");
//    Serial.println(w); 

//    Serial.print("dX: ");
//    Serial.print(dX);
//    Serial.print(", dY: ");
//    Serial.print(dY);
//    Serial.print(", w:");
//    Serial.print(w);
//    Serial.print(", vLeft: ");
//    Serial.print(velocityLeft);
//    Serial.print(": vRight:");
//    Serial.println(velocityRight); 

//      Serial.print("lTicks: ");
//      Serial.print(leftCountNew);
//      Serial.print(", rTicks: ");
//      Serial.println(rightCountNew);
      }
      else
      {
        getPosition();
        
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
  
  goalHeading = atan2(dY,dX); //Heading required to reach the goal
  errorHeading = goalHeading - heading; //Error with current heading
  errorHeading = atan2(sin(errorHeading),cos(errorHeading)); 
  
  //With time
  timeCurrent = micros();

  /* Serial.print("Tcurrent: ");
  Serial.print(timeCurrent);
  Serial.print(", Told");
  Serial.println(timeOld); */
  
  dt = timeCurrent - timeOld; //Time difference since last execution
  //dt = dt / 1000; //Convert to seconds

  //Vo = distanceCenter / dt; //Calculate current speed of robot in m/s
  float dX2 = pow(dX, 2);
  float dY2 = pow(dY, 2);
  Vo = sqrt(dY2 + dX2);
  
  errorP = errorHeading; //Proportional error
  errorI = errorI + (errorHeading * dt); //Integral error
  errorD = (errorHeading - errorPrevious) / dt; //Derivative error
  
  errorPrevious = errorHeading; //Store previous error
  timeOld = timeCurrent; //Reset time
  
 //w = (Kp * errorP) + (Ki * errorI) + (Kd * errorD); //Result of PID
   w = (Kp * errorP); //P controller only
  
  velocityLeft = ((2*Vo - w*wheelBase) / (2*wheelRadius)); //Calculated velocity of left wheel
  velocityRight = ((2*Vo + w*wheelBase) / (2*wheelRadius)); //Calculated velocity of right wheel
}

void getPosition()
{
  leftCount = leftCountNew - leftCountOld; //Difference in tick count since last call for left wheel
  rightCount = rightCountNew - rightCountOld; //Difference in tick count since last call for right wheel
  
  distanceLeft = 2 * pi * leftCount / wheelTicks; //Distance travelled by left wheel
  distanceRight = 2 * pi * rightCount / wheelTicks; //Distance travelled by right wheel
  distanceCenter = (distanceLeft + distanceRight) / 2; //Distance travelled by center of robot
  
  positionX = positionXOld + distanceCenter * cos(headingOld); //Current X position
  positionY = positionYOld + distanceCenter * sin(headingOld); //Current Y position
  heading = headingOld + ((distanceRight - distanceLeft) / wheelBase); //Current heading
  heading = atan2(sin(heading),cos(heading));

  //Reset previous tick count
  leftCountOld = leftCountNew; 
  rightCountOld = rightCountNew;

  //Reset previous positions value
  positionXOld = positionX;
  positionYOld = positionY;
  headingOld = heading;
}

//Convert velocity given by PID to value usable by the servos
//HIGH CHANCE OF THIS FUNCTION BEING THE PROBLEM
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

  //Get porper velocity in the wheels
  velocityLeft = velocityLeft + 90;
  velocityRight = 90 - velocityRight;

  
   
  //If the wheel is suppose to go in reverse, make it stop as to not mess with the odometry ticks
  if(velocityLeft <= 90)
  {
    velocityLeft = 90;
  }
  else if (velocityRight >=90)
  {
    velocityRight = 90;
  }
  
  leftServo.write(velocityLeft);
  rightServo.write(velocityRight);

    /*Serial.print(", vLeft: ");
    Serial.print(velocityLeft);
    Serial.print(": vRight:");
    Serial.println(velocityRight); */
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
//void stopMotor()
//{
//  while(true)
//  {
//    leftServo.writeMicroseconds(1500);
//    rightServo.writeMicroseconds(1500);
//  }
//}

/*
void getInitialPosition()
{
  //Use protocol go get initial position from the raspberry pi
}

void findFirstTag()
{
  //Either sweep servo motor until stop command from pi and send camera servo angle
  //OR
  //Get specific angle from pi to move servo to
}

*/

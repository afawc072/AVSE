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
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(13, OUTPUT);
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

        bool reached = true;
        
        // SIMON FUNCTION -> format : reached = moveRobotTo(x,y);
        test_LED(round(x+y)); //Temporary test to see if vector was properly received
        
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


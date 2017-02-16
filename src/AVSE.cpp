/*******************************************************************************
*
* PROJECT: AUTONOMOUS VEHICULE IN A STRUCTURED ENVIRONMENT
*
* SECTION: Prototype
*
* AUTHOR: Alexandre Fawcett and Jean-Sebastien Fiset
*
* DESCRIPTION:
*
*	 Prototype main function
*
* NOTES:
*
*
*
********************************************************************************/
/** @file AVSE.cpp*/

#include "AVSE.h"

using namespace std;



/*******************************************************************************
 * tagDetection
 *
 *    This function is used to detect a tag in the environment. Using the the 
 * constants "CAMANGLE_START", "CAMANGLE_INCREMENT" and "CAMANGLE_END", a sweep
 * of the environment is maded by the camera (mounted on the Arduino side) until
 * a Tag is found. The Tag detection is made through the ___________ of the 
 * computervision object. The variable passed by reference (TagID, X and Z distances 
 * from the Tag to the Robot and the camMarkerAngle) are filled in the computervision
 * function and return to the main function in order for the robot to localize itself
 * within the grid.
 *    The function returns true if a tag is found during the sweep.
 *
 *  @param [in]   apProtocol
 *  @param [in]   arTagID
 *  @param [in]   arxCam
 *  @param [in]   arzCam
 *  @param [in]   arAngleCamMarker
 *  @param [in]   arCamServoAngle
 *
 *  @return    foundTag
 *
 *******************************************************************************/
static bool tagDetection(Protocol *apProtocol, int &arTagID, double &arxCam, double &arzCam, double &arAngleCamMarker, double &arCamServoAngle)
{
   // Initialize required variable/objects
   errorType error;
   // ComputerVision compVision;

   bool foundTag = false;
   double camServoAngle = CAMANGLE_START;

   // Loop until either a Tag is found or the camera has done a full sweep
   while( !foundTag && camServoAngle <= CAMANGLE_END )
   {
      // Send CAMANGLE message to Arduino to set the camera angle
      if( apProtocol->send(CAMANGLE, to_string(camServoAngle), error) )
      {
         command cmdRcvd;
         string infoRcvd;

	 // Receive the feedback from the Arduino
         if( apProtocol->receive(NB_TRIES_CAMANGLE, DELAY_CAMANGLE, cmdRcvd, infoRcvd, error) )
         {
            if( cmdRcvd == END )
            {
               // Call computerVision function to search for a Tag
//               foundTag = compVision.findATag(artagId, arxCam, arzCam, arAngleCamMarker));
               if( !foundTag )
               {
                  camServoAngle += CAMANGLE_INCREMENT;
               }
            }
            else // cmdRcvd != END
            {
               cout << "ERROR : Message received was not END" << endl;
            }
         }
         else // Error in receiving the message back from Arduino
         {
            cout << PROTOCOL_ERR[error] << endl;
         }
      }
      else // Error in sending the position vector
      {
         cout << PROTOCOL_ERR[error] <<endl;
      }
   }

   // Return the flag indicating that a tag was found or not
   return foundTag;
}




/*******************************************************************************
 * moveRobot
 *
 *
 *  @param [in]   apModel
 *  @param [in]   apProtocol
 *
 *
 *******************************************************************************/
static void moveRobot(NavigationModel * apModel, Protocol * apProtocol)
{
   errorType error;
   vector<float> firstPosition;

   // Send the first position vector to the Arduino
   if( apModel->nextPositionVector(&firstPosition) )
   {
      string infoNext = to_string(firstPosition[0]);
      infoNext += ",";
      infoNext += to_string(firstPosition[1]);

      cout << "Sending position vector: " << infoNext <<endl;

      if( !apProtocol->send(NEXT, infoNext, error) )
      {
	 cout << PROTOCOL_ERR[error] << endl;
      }

   }


   // Loop until destination is reached
   while( !apModel->destinationIsReached() )
   {
      command cmdRcvd;
      string infoRcvd;

      // Receive feedback from Arduino
      if( apProtocol->receive(NB_TRIES_NEXT, DELAY_NEXT, cmdRcvd, infoRcvd, error) )
      {

         if( cmdRcvd == REACHED )
         {
	    // Extract obstacle distances from infoRcvd
            vector<float> obstDistances;
	    stringstream ss(infoRcvd);

	    while(ss)
	    {
	      string temp;
	      if( !getline(ss, temp, ',') ) break;
	      obstDistances.push_back(stof(temp));
	    }

	    // Add obstacle
	    apModel->addObstacle(obstDistances);

	    // Send next position vector
	    vector<float> nextPosition;

	    // If there is a position vector, it is sent to the Arduino
	    if( apModel->nextPositionVector(&nextPosition) )
	    {
	       string infoNext = to_string(nextPosition[0]);
	       infoNext += ",";
	       infoNext += to_string(nextPosition[1]);

	       cout << "Sending position vector: " << infoNext <<endl;
	       // Send the next position vector to the Arduino
	       if( !apProtocol->send(NEXT, infoNext, error) )
	       {
         	  cout << PROTOCOL_ERR[error] << endl;
               }
	    }
         }

 	 else if( cmdRcvd == STOP )
	 {
	    // TO-DO
            apModel->clearPath();
            break;
	 }

	 // Unknown command
         else
         {
            cout << "ERROR: Command received is : " << PROTOCOL_DICT[cmdRcvd] <<endl;
         }
      }

      else
      {
	 cout << PROTOCOL_ERR[error] << endl;
      }
   }
}





/*******************************************************************************
 * main
 *
 *
 *
 *
 *******************************************************************************/
int main(int argc, const char **argv)
{
   errorType error;

   //Setup Grid
   Grid myGrid(GRID_FILE_PATH);
   myGrid.printGrid();

   // Initialize Navigation object
   NavigationModel model = NavigationModel(&myGrid);

   // Open connection to Arduino
   Protocol protocol;

   if( !protocol.init(error) )
   {
      cout << PROTOCOL_ERR[error] <<endl;
   }

   // Initialization of the protocol has succeeded
   else
   {
      // Main loop
      while( true )
      {

         //1. SET THE FINAL DESTINATION
         int goal_ix;
         cout << "DESTID-> ";

         do
         {
            cin >> goal_ix; 		 		// READ FROM LCD
         } while( !model.setDestination(goal_ix) );


         // 2. LOCALIZATION AND ORIENTATION

         // 2.1 CAMERA-TAG RELATION
         int tagID;
         double xTag, zTag, angleCamMarker, camServoAngle;


         // Attempt to detect the first tag
         if( tagDetection(&protocol, tagID, xTag, zTag, angleCamMarker, camServoAngle) )
         {
            // 2.2 ROBOT-GRID RELATION

            // Initialize the robot
            if( model.localizeRobotInGrid(tagID, xTag, zTag, angleCamMarker, camServoAngle) )
	    {

               // 3. FIND POTENTIAL PATH TO DESTINATION
               if( model.calculatePathToDest() )
	       {
                  model.print();

                  // 4. MOVE TOWARD GOAL
	          moveRobot(&model, &protocol);

	          // 5. CHECK IS DESTINATION WAS REACHED
                  if( model.destinationIsReached() )
                  {

                     // Call openCV function to detect a tag and localize the robot in the grid
	             if( tagDetection(&protocol, tagID, xTag, zTag, angleCamMarker, camServoAngle) )
    	             {
                        if( !model.localizeRobotInGrid(tagID, xTag, zTag, angleCamMarker, camServoAngle) )
			{
			   cout << "Unable to localize robot to apply a correction. Robot should be close to destination"<<endl;
			}
                     }
	             else
	             {
	   	        cout << "Didn't find a tag at final destination, no correction can be applied" << endl;
	             }

	             /* If the updated position after detecting a tag does not match
	                the final destination, a correction is applied */
                     if( !model.destinationIsReached() )
                     {
	                cout << "CORRECTION APPLIED" << endl;

			if( model.calculatePathToDest() )
		        {
                           moveRobot(&model, &protocol);
                     	}
			else
			{
			   cout << "Unable to calculate path to apply a correction. Robot should be close to destination"<<endl;
			}
		     }

                     printf("FINAL\n");				// WRITE TO LCD

                     model.print();
                     cout << "Now resetting the grid" <<endl;
	             myGrid.resetGrid();
                  }
	          else
 	          {
		      // Destination was not reach for some reason (user/Arduino forced the robot to stop with STOP command)
	          }
               }
	       else // Calculation of the path failed
	       {
		  cout << "Unable to calculate the path. Try another destination"<<endl;
	       }
	   }
	   else // Localization of the robot failed
	   {
		cout <<"Unable to locate robot. Try to move it closer to a marker"<<endl;
	   }
	 }
	 else // no tag were found
         {
            cout << "NOTAG" <<endl; 			// WRITE TO LCD
         }
      }
   }
}

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
 *  @param [in]   aMaxTagID
 *  @param [in]   arTagID
 *  @param [in]   arxCam
 *  @param [in]   arzCam
 *  @param [in]   arAngleCamMarker
 *  @param [in]   arCamServoAngle
 *
 *  @return    foundTag
 *
 *******************************************************************************/
static bool tagDetection(Protocol *apProtocol, int aMaxTagID, int &arTagID, double &arxCam, double &arzCam, double &arAngleCamMarker, double &arCamServoAngle)
{
   fprintf(file, "Tag Dectection Process Initiated\n");
   fflush(file);

   // Initialize required variable/objects
   errorType error;
   computervision compVision(CAM_PARAM);

   bool foundTag = false;
   arCamServoAngle = CAMANGLE_START;

   // Loop until either a Tag is found or the camera has done a full sweep
   while( !foundTag && arCamServoAngle <= CAMANGLE_END )
   {
      fprintf(file,"Camera Angle set to: %.02f\n",arCamServoAngle);  
      fflush(file);

      // Send CAMANGLE message to Arduino to set the camera angle
      if( apProtocol->send(CAMANGLE, to_string(arCamServoAngle), error) )
      {
         command cmdRcvd;
         string infoRcvd;

	 // Receive the feedback from the Arduino
         if( apProtocol->receive(NB_TRIES_CAMANGLE, DELAY_CAMANGLE, cmdRcvd, infoRcvd, error) )
         {
            if( cmdRcvd == END )
            {
               // Call computerVision function to search for a Tag
               foundTag = compVision.detectTag(aMaxTagID, arTagID, arxCam, arzCam, arAngleCamMarker);
               if( !foundTag )
               {
                  arCamServoAngle += CAMANGLE_INCREMENT;
               }
            }
            else // cmdRcvd != END
            {
               cout << "ERROR : Message received was not END" << endl;
               fprintf(file, "ERROR : Message received was not END\n");
	       fflush(file);
            }
         }
         else // Error in receiving the message back from Arduino
         {
            cout << PROTOCOL_ERR[error] << endl;
	    fprintf(file,"Error Receiving the message back fomr the Arduino: %s\n", PROTOCOL_ERR[error].c_str());
	    fflush(file);
         }
      }
      else // Error in sending the position vector
      {
         cout << PROTOCOL_ERR[error] <<endl;
	 fprintf(file,"Error Sending Position Vector: %s\n", PROTOCOL_ERR[error].c_str());
         fflush(file);
      }
   }

   // Return the flag indicating that a tag was found or not
   fprintf(file, "Tag Detection Process Terminated\n");
   fflush(file);

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
   command cmdRcvd;
   string infoRcvd;
   bool possiblePathFlag = true;

   fprintf(file, "moveRobot Function Initiated\n");
   fflush(file);

   // Send the first position vector to the Arduino
   if( apModel->nextPositionVector(&firstPosition) )
   {
      string infoNext = to_string(firstPosition[0]);
      infoNext += ",";
      infoNext += to_string(firstPosition[1]);

      cout << "Sending position vector: " << infoNext <<endl;
      fprintf(file,"Sending position vector: %s\n", infoNext.c_str());
      fflush(file);

      if( !apProtocol->send(NEXT, infoNext, error) )
      {
	 cout << PROTOCOL_ERR[error] << endl;
	 fprintf(file,"Protocol Error: %s\n", PROTOCOL_ERR[error].c_str());
         fflush(file);
      }

   }


   // Loop until destination is reached
   while( !apModel->destinationIsReached() && possiblePathFlag)
   {

      // Receive feedback from Arduino
      if( apProtocol->receive(NB_TRIES_NEXT, DELAY_NEXT, cmdRcvd, infoRcvd, error) )
      {

	 // Position was reached without problem, send position vector right away
         if( cmdRcvd == REACHED )
         {
            // Update robot position in the grid
            apModel->moveRobotToNextPosition();

            // Send next position vector right away
            vector<float> nextPosition;

            // If there is a position vector, it is sent to the Arduino
            if( apModel->nextPositionVector(&nextPosition) )
            {
               string infoNext = to_string(nextPosition[0]);
               infoNext += ",";
               infoNext += to_string(nextPosition[1]);

               cout << "Sending position vector: " << infoNext <<endl;
               fprintf(file, "Sending position vector: %s\n", infoNext.c_str());
               fflush(file);

               // Send the next position vector to the Arduino
               if( !apProtocol->send(NEXT, infoNext, error) )
               {
                  cout << PROTOCOL_ERR[error] << endl;
                  fprintf(file,"Protocol Error: %s\n", PROTOCOL_ERR[error].c_str());
                  fflush(file);

               }
            }

            // Extract obstacle distances from infoRcvd
            vector<float> obstDistances;
            stringstream ss(infoRcvd);
            fprintf(file,"Distance Received: %s\n",infoRcvd.c_str());
            fflush(file);

            while(ss)
            {
              string temp;
              if( !getline(ss, temp, ',') ) break;
              obstDistances.push_back(stof(temp));
            }

	    // Add obstacle
	    if( !apModel->addObstacle(obstDistances) )
            {
               possiblePathFlag = false;
            }

        }

        // An obstacle was detected too close, analyze data before sending next vector
        else if( cmdRcvd == STOP )
        {
	   fprintf(file, "Received STOP command from Arduino (a sensor deteced an obstacle too close). Distances received: %s\n",infoRcvd.c_str());
           fflush(file);

           // Extract obstacle distances from infoRcvd
           vector<float> obstDistances;
           stringstream ss(infoRcvd);
           fprintf(file,"Distance Received: %s\n",infoRcvd.c_str());
           fflush(file);

           while(ss)
           {
             string temp;
             if( !getline(ss, temp, ',') ) break;
             obstDistances.push_back(stof(temp));
           }

           // Add obstacle
           apModel->addObstacle(obstDistances);

           // Send next position vector right away
           vector<float> nextPosition;

           // If there is a position vector, it is sent to the Arduino
           if( apModel->nextPositionVector(&nextPosition) )
           {
              string infoNext = to_string(nextPosition[0]);
              infoNext += ",";
              infoNext += to_string(nextPosition[1]);

              cout << "Sending position vector: " << infoNext <<endl;
              fprintf(file, "Sending position vector: %s\n", infoNext.c_str());
              fflush(file);
              // Send the next position vector to the Arduino
              if( !apProtocol->send(NEXT, infoNext, error) )
              {
                 cout << PROTOCOL_ERR[error] << endl;
                 fprintf(file,"Protocol Error: %s\n", PROTOCOL_ERR[error].c_str());
                 fflush(file);
              }
           }
        }

	// Error on the Arduino side
        else if( cmdRcvd == ERROR )
        {
           fprintf(file, "Received error command from Arduino with data: %s\n",infoRcvd.c_str());
	   fflush(file);
	}

        // Unknown command
        else
        {
           cout << "ERROR: Command received is : " << PROTOCOL_DICT[cmdRcvd] <<endl;
	   fprintf(file, "ERROR: Command received was : %s\n",PROTOCOL_DICT[cmdRcvd].c_str());
	   fflush(file);
        }
      }

      else // Error in protocol receive function
      {
	 cout << PROTOCOL_ERR[error] << endl;
         fprintf(file,"Protocol Error: %s\n", PROTOCOL_ERR[error].c_str());
         fflush(file);
      }
   }
   if( possiblePathFlag )
   {
      if( apProtocol->receive(NB_TRIES_NEXT, DELAY_NEXT, cmdRcvd, infoRcvd, error) )
      {
         if( cmdRcvd != REACHED )
         {
           cout << "Error when receiving final REACHED message. The message received was: [" <<PROTOCOL_DICT[cmdRcvd]<<":"<<infoRcvd<<"]"<<endl; 
           fprintf(file, "Received error command from Arduino with data: [%s:%s]\n",PROTOCOL_DICT[cmdRcvd].c_str(),infoRcvd.c_str());
           fflush(file);
         }
      }
      else
      {
         cout << "Error in receive function for the final REACHED message" <<endl;
         fprintf(file, "Error in receive function for the final REACHED message\n");
         fflush(file);
      }
   }

}


/*******************************************************************************
 * LOGFILE
 *
 *
 * CHRISTOPH - https://stackoverflow.com/questions/1425227/how-to-create-files-named-with-current-time 
 *
 *******************************************************************************/
FILE *logfile(void)
{
    static char name[32];
    time_t now = time(0);
    strftime(name, sizeof(name), "logs/log_%Y_%m_%d_%H-%M.txt", localtime(&now));
    return fopen(name, "w+");
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

   file = logfile();
   fprintf(file,"!!INITIATION!!\n");
   fflush(file);
   
   //Setup Grid
   Grid myGrid(GRID_FILE_PATH);
   myGrid.printGrid();   
   myGrid.printGrid(file);

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

         do
         {
            cout << "DESTID-> ";
            cin >> goal_ix; 		 		// READ FROM LCD
         } while( !model.setDestination(goal_ix) );


         // 2. LOCALIZATION AND ORIENTATION

         // 2.1 CAMERA-TAG RELATION
         int tagID;
         double xTag, zTag, angleCamMarker, camServoAngle;
         int maxTagID = myGrid.getTagPositions().size();

         // Attempt to detect the first tag
         if( tagDetection(&protocol, maxTagID, tagID, xTag, zTag, angleCamMarker, camServoAngle) )
         {
	    cout << "Tag " << tagID << ", xTag: "<< xTag << ", zTag: "<< zTag << ", angle " <<angleCamMarker << ",camangle " << camServoAngle<<endl;
	    fprintf(file, "Tag %d, xTag: %.02f, zTag; %.02f, angle: %.02f, camangle: %.02f\n",tagID,xTag,zTag,angleCamMarker,camServoAngle);
   	    fflush(file);
            // 2.2 ROBOT-GRID RELATION

            // Initialize the robot
            if( model.localizeRobotInGrid(tagID, xTag, zTag, angleCamMarker, camServoAngle) )
	    {

               // 3. FIND POTENTIAL PATH TO DESTINATION
               if( model.calculatePathToDest() )
	       {
                  model.print(file);
	          model.print();
                  // 4. MOVE TOWARD GOAL
	          moveRobot(&model, &protocol);

	          // 5. CHECK IS DESTINATION WAS REACHED
                  if( model.destinationIsReached() )
                  {

                     // Call openCV function to detect a tag and localize the robot in the grid
	             if( tagDetection(&protocol, maxTagID, tagID, xTag, zTag, angleCamMarker, camServoAngle) )
    	             {
                        if( !model.localizeRobotInGrid(tagID, xTag, zTag, angleCamMarker, camServoAngle) )
			{
			   cout << "Unable to localize robot to apply a correction. Robot should be close to destination"<<endl;
                           fprintf(file,"Unable to localize robot to apply a correction. Robot should be close to destination\n");
                           fflush(file);
			}
                     }
	             else
	             {
	   	        cout << "Didn't find a tag at final destination, no correction can be applied" << endl;
                        fprintf(file,"Didn't find a tag at final destination, no correction can be applied\n");
                        fflush(file);
	             }

	             /// If the updated position after detecting a tag does not match
	             //   the final destination, a correction is applied 
                     if( !model.destinationIsReached() )
                     {
	                cout << "CORRECTION APPLIED" << endl;
                        fprintf(file,"CORRECTION APPLIED\n");
                        fflush(file);

			if( model.calculatePathToDest() )
		        {
                           model.print(file);
	                   model.print();
                           moveRobot(&model, &protocol);
                     	}
			else
			{
			   cout << "Unable to calculate path to apply a correction. Robot should be close to destination"<<endl;
                           fprintf(file, "Unable to calculate path to apply a correction. Robot should be close to destination\n");
                           fflush(file);
			}
		     }

                     fprintf(file,"FINAL\n");				// WRITE TO LCD
	             fflush(file);
                     cout << "Now resetting the grid" <<endl;
	             fprintf(file, "Now Resetting the Grid");
		     fflush(file);
		     model.print(file);
		     model.print();
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
	 	  fprintf(file,"Unable to calculate the path. Try another destination\n");
		  fflush(file);
	       }
	   }
	   else // Localization of the robot failed
	   {
		cout <<"Unable to locate robot. Try to move it closer to a marker"<<endl;
	 	fprintf(file, "Unable to locate robot. Try to move it closer to a marker\n");
		fflush(file);
	   }
	 }
	 else // no tag were found
         {
            cout << "NOTAG" <<endl;
	    fprintf(file,"NOTAG\n");
	    fflush(file); 			// WRITE TO LCD
         }
      }
   }
fclose(file);
}

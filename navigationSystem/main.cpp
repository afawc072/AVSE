#include "NavSystemCommon.h"
#include "Grid.h"
#include "NavigationModel.h"

using namespace std;


static void moveRobot(NavigationModel * model)
{
   vector<float> firstPosition;
   if(model->nextPositionVector(&firstPosition))
   {
      printf("NEXT (%.2f, %.2f)\n",firstPosition[0],firstPosition[1]);		// WRITE TO ARD
   }

   cout << "Moving the robot, type R if cell is reached, type ADD followed by (on the next line) <SensorID> <angle> <distance_cm> to add a obstacle or STOP" <<endl;

   while(!model->destinationIsReached())
   {
      vector<float> nextPosition;
      string feedback;
      cin >> feedback;								// READ FROM ARD

      vector<string> obstacles;

      if(feedback == "ADD")
      {
         int sensorID;
         float angle;
         float distance;

         //Input the information regarding the new obstacle
         cin >> sensorID >> angle >> distance;
         model->addObstacle(sensorID, angle, distance);
      }
      else if(feedback == "R")
      {
          if(model->nextPositionVector(&nextPosition))
          {
             printf("NEXT (%.2f, %.2f)\n",nextPosition[0],nextPosition[1]);	//WRITE TO ARD
          }
      }
      else if(feedback == "STOP")
      {
	 break;
      }
      else
      {
          //printf("Invalid keyword, try either ADD (followed by obstacle info ) or REACHED\n");
      }
   }
}



int main(int argc, const char **argv)
{

   //Setup Grid
   Grid myGrid;
   myGrid.printGrid();

   // Initialize Navigation object
   NavigationModel model = NavigationModel(&myGrid);


   // Main loop
   while(true)
   {

      //1. SET THE FINAL DESTINATION
      int goal_ix;
      cout << "DESTID-> ";

      do
      {
         cin >> goal_ix; 				// READ FROM LCD
      } while(!model.setDestination(goal_ix));

      cout << "STARTCAMSERVO" << endl; 			// WRITE TO ARD


      // 2. LOCALIZATION AND ORIENTATION

      // 2.1 CAMERA-TAG RELATION
      bool foundTag;
      int tagID;
      double xdist;
      double ydist;

      // Call openCV function
      //foundTag = findATag(&tagID, &xdist, &ydist);
      cout << "... (using ARToolKit) ... Oh! x(col) and y(row) distance to tag and tagID found and hardcoded here" <<endl;
      foundTag=true;
      tagID=2;
      xdist=-10;
      ydist=-18;

      if(foundTag)
      {
         cout << "STOPCAMSERVO"<<endl; 			//WRITE TO ARD

         // 2.2 ROBOT-GRID RELATION
         cout << "CAMANGLE-> ";
         double camServoAngle;
         cin >> camServoAngle; 				// READ FROM ARD

         // Initialize the robot
         model.localizeRobotInGrid(tagID, xdist, ydist, camServoAngle);

         // 3. FIND POTENTIAL PATH TO DESTINATION
         model.calculatePathToDest();
         model.print();

         // 4. MOVING TOWARD GOAL
	 moveRobot(&model);

         if(model.destinationIsReached())
         {
            // Call openCV function

            /*foundTag = findATag(&tagID, &xdist, &ydist);
            localizeRobotInGrid(model, tagID, xdist, ydist, 0);

            if(!model.destinationIsReached())
            {
	       printf("CORRECTION APPLIED\n");
               moveRobot(&model);
            }*/

            printf("FINAL\n");				// WRITE TO LCD

            model.print();
            myGrid.resetGrid();
         }
      }
      else
      {
         cout << "NOTAG" <<endl; 			// WRITE TO LCD
      }
   }
}

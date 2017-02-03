/*******************************************************************************
*
* PROJECT: AUTONOMOUS VEHICULE IN A STRUCTURED ENVIRONMENT
*
* SECTION: Navigation System
*
* AUTHOR: Jean-Sebastien Fiset
*
* DESCRIPTION:
*
*	 Test main function to test the navigation system section
*
* NOTES:
*
*
*
********************************************************************************/
/** @file main.cpp*/

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

   cout << "Moving the robot, type R if cell is reached, type ADD followed by (on the next line) <rowIndex> <colIndex> to add an obstacle or STOP" <<endl;

   while(!model->destinationIsReached())
   {
      vector<float> nextPosition;
      string feedback;
      cin >> feedback;								// READ FROM ARD

      vector<string> obstacles;

      if(feedback == "ADD")
      {
         float x,y;

         vector<float> obst_distances;

         //Input the information regarding the new obstacle
         cin >> x >> y;
         obst_distances.push_back(x);
         obst_distances.push_back(y);

         model->addObstacle(obst_distances);
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
	 model->clearPath();
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

   // Open connection to arduino

   // Main loop
   while(true)
   {

      //1. SET THE FINAL DESTINATION
      int goal_ix;
      cout << "DESTID-> ";

      do
      {
         cin >> goal_ix; 		 		// READ FROM LCD
      } while(!model.setDestination(goal_ix));

      cout << "STARTCAMSERVO" << endl; 			// WRITE TO ARD


      // 2. LOCALIZATION AND ORIENTATION

      // 2.1 CAMERA-TAG RELATION
      bool foundTag;
      int tagID;
      double xdist;
      double zdist;

      // Call openCV function
      //foundTag = findATag(&tagID, &xdist, &zdist);
      cout<<"Enter <tagID> <xdist> <zdist>:"<<endl;
      cin >> tagID >> xdist >> zdist ;

      foundTag=true;

      if(foundTag)
      {
         cout << "STOPCAMSERVO"<<endl; 			//WRITE TO ARD

         // 2.2 ROBOT-GRID RELATION
         cout << "CAMANGLE-> ";
         double camServoAngle;
         cin >> camServoAngle; 				// READ FROM ARD

         // Initialize the robot
         model.localizeRobotInGrid(tagID, xdist, zdist, camServoAngle);

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

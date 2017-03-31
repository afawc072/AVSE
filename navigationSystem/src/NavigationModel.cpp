/*******************************************************************************
*
* PROJECT: AUTONOMOUS VEHICULE IN A STRUCTURED ENVIRONMENT
*
* SECTION: Navigation System
*
* AUTHOR: Alexandre Fawcett & Jean-Sebastien Fiset 
*
* DESCRIPTION:
*
*	Navigation Model Function Implementation
*
* NOTES:
*
*
*
********************************************************************************/
/** @file NavigationModel.cpp*/

#include "NavigationModel.h"



/*******************************************************************************
 * NavigationModel
 *
 * 	Default constructor
 *
 *******************************************************************************/
NavigationModel::NavigationModel()
{
   printf("NavigationMode - WARNING: You should send a grid in the constructor. This section should be fixed.\n");
   Grid lGrid;
   mrGrid=&lGrid;

   mKnownDestination=false;
}




/*******************************************************************************
 * NavigationModel
 *
 * 	Constructor taking a pointer of an already initialized Grid object
 *
 * @param  [in]  apGrid
 *
 *******************************************************************************/
NavigationModel::NavigationModel(Grid *apGrid)
{
   mrGrid = apGrid;
   mKnownDestination=false;
}




/*******************************************************************************
 * setDestination
 *
 * 	Sets the desired final destination of the model. Changes the flag to indicate
 * that the destination is known and a path can be calculted.
 *
 * @param  [in]  aDestinationID
 *
 * @return       validDest
 *
 *******************************************************************************/
bool NavigationModel::setDestination(int aDestinationID)
{
   bool validDest = false;

   if( aDestinationID >=0 && aDestinationID < (int) mrGrid->getDestPositions().size() )
   {
      mFinalPosition = mrGrid->getDestPositions()[aDestinationID];
      mKnownDestination = true;
      validDest = true;
   }

   return validDest;
}




/*******************************************************************************
 * destinationIsReached
 *
 * 	Returns true if the robot's current position match the final desired position
 *
 * @return       destinationReached
 *
 *******************************************************************************/
bool NavigationModel::destinationIsReached()
{
   bool destinationReached = false;

   if( mRobot.mCurrentPosition.row == mFinalPosition.row &&
      mRobot.mCurrentPosition.column == mFinalPosition.column )
   {
      destinationReached = true;
   }

   return destinationReached;
}




/*******************************************************************************
 * clearPath
 *
 *      Clears the path stack and change the state of the path cell from P to F.
 *
 *
 *******************************************************************************/
void NavigationModel::clearPath()
{
   while( !mPathToDest.empty() )
   {
      mrGrid->removePathCell(mPathToDest.top());
      mPathToDest.pop();
   }
}




/*******************************************************************************
 * updateRobotOrientation
 *
 * 	Sets the robot's orientation
 *
 * @param  [in]  aOrientation
 *
 *******************************************************************************/
void NavigationModel::updateRobotOrientation(double aOrientation)
{
   mRobot.mOrientation=aOrientation;
}




/*******************************************************************************
 * updateRobotPosition
 *
 *      Sets the robot's position (row and column)
 *
 * @param  [in]  aNewRow
 * @param  [in]  aNewCol
 *
 *******************************************************************************/
void NavigationModel::updateRobotPosition(int aNewRow, int aNewCol)
{
   mRobot.updatePosition(aNewRow, aNewCol);
}





/*******************************************************************************
 * localizeRobotInGrid
 *
 *      Knowing the tagID that was found by the camera and the x and z distance of
 * the robot in respect to the tag, we can localize the robot in the grid. The first
 * step is to find the transformation matrix from the grid to the tag. The tag
 * frame is known by finding out where it is pointing (right, left, down, up) according
 * to the grid. With that information, we can use the right transformation matrix and
 * find the row and column where the robot is located.
 *
 * @param  [in]  aTagID
 * @param  [in]  aXdist_cm
 * @param  [in]  aYdist_cm
 * @param  [in]  aCamMarkerAngle
 * @param  [in]  aCamServoAngle
 *
 * @return validLocalization
 *
 *******************************************************************************/
bool NavigationModel::localizeRobotInGrid(int aTagID, double aXdist_cm, double aZdist_cm, double aCamMarkerAngle, double aCamServoAngle)
{
   bool validLocalization = false;
   double cameraOrientation;

   // Find the angle from the tag to the robot starting from the x axis of the Tag
   double angleToRobot = atan2(aZdist_cm, aXdist_cm)*180/PI;
   printf("angleToRobot : %.02f\n",angleToRobot);

   if( aTagID < 1 || aTagID > (int) (mrGrid->getTagPositions().size()))
   {
      cout << "Invalid Tag ID"<<endl;
      return false;
   }
   // Get the tag position (tagID starts at 1 and vector index at 0)
   Position tagPos = mrGrid->getTagPositions()[aTagID-1];

   // Initialize vector to model the 4 possibile direction (right, left, down and up)
   vector<int> row4dir={0, 0, 1,-1};
   vector<int> col4dir={1,-1, 0, 0};

   unsigned int i;

   // We first find whether the tag is facing to the right, left, down or up
   for( i = 0 ; i <row4dir.size() ; i++ )
   {
      if( mrGrid->isBuffer(tagPos.row + row4dir[i], tagPos.column + col4dir[i]) )
      {
         break;
      }
   }

   // Transformation matrix from grid to the tag
   int QGtoT[4][4]=  {{ 0 , 0 , 0 , tagPos.row*GRID_CM},
                      { 0 , 0 , 0 , tagPos.column*GRID_CM},
                      { 0 , 1 , 0 , 0},
                      { 0 , 0 , 0 , 1}};;


   // The base transformation matrix is modified depending on the orientation of the tag (4 cases)
   switch(i)
   {
      case 0: // Tag is facing to the right
	 QGtoT[0][0] =-1;
	 QGtoT[1][2] = 1;
//         cameraOrientation = aCamMarkerAngle - angleToRobot + 180;
	   cameraOrientation = aCamMarkerAngle + angleToRobot;
 	   printf("Tag facing right.\n");
      break;

      case 1: // Tag is facing to the left
         QGtoT[0][0] = 1;
         QGtoT[1][2] =-1;
//	 cameraOrientation = aCamMarkerAngle - angleToRobot;
	 cameraOrientation = aCamMarkerAngle + angleToRobot - 180;
           printf("Tag facing left.\n");
      break;

      case 2: // Tag is facing down
         QGtoT[1][0] = 1;
         QGtoT[0][2] = 1;
	 cameraOrientation = aCamMarkerAngle - angleToRobot + 90;
         printf("Tag facing down.\n");
      break;

      case 3: // Tag is facing up
         QGtoT[1][0] =-1;
         QGtoT[0][2] =-1;
	 cameraOrientation = aCamMarkerAngle - angleToRobot - 90;
         printf("Tag facing up.\n");
      break;

      default:
	// Do nothing
      break;
   }
 
   // Calculate and set the robot's orientation in respect to the grid
   double robotOrientation = -(cameraOrientation - aCamServoAngle);
   updateRobotOrientation(robotOrientation);
   cout << "Camera orientation is " << -cameraOrientation << endl;

   // Calculate shift from the camera to the center of the robot to adjust the localization
   float xCorrectionCamCenter = -CAM_RCENTER_DIST*sin(robotOrientation*PI/180);
   float yCorrectionCamCenter = -CAM_RCENTER_DIST*cos(robotOrientation*PI/180);

   // Calculate the robot position
   int robotRow = round((QGtoT[0][0]*aXdist_cm + QGtoT[0][2]*aZdist_cm + QGtoT[0][3] + xCorrectionCamCenter)/GRID_CM);
   int robotCol = round((QGtoT[1][0]*aXdist_cm + QGtoT[1][2]*aZdist_cm + QGtoT[1][3] + yCorrectionCamCenter)/GRID_CM);

   cout << "Tag detected is at " <<tagPos.row<<","<<tagPos.column<<", robot is at " <<robotRow <<","<<robotCol<<endl;

   if( !mrGrid->isOccupied(robotRow, robotCol) || mrGrid->isNextToFree(robotRow, robotCol) )
   {
      validLocalization = true;
      updateRobotPosition(robotRow, robotCol);
   }

   return validLocalization;
}



/*******************************************************************************
 * moveRobotToNextPosition
 *
 *      Changes the robot position to the next cell and remove the path cell from
 * the grid and the stack.
 *
 *
 *******************************************************************************/
bool NavigationModel::moveRobotToNextPosition()
{
   bool validUpdate = false;

   if( !mPathToDest.empty() )
   {
      validUpdate = true;

      // Update position and orientation of the robot
      Position newPos = mPathToDest.top();
      updateRobotOrientation(atan2((newPos.row - mRobot.mCurrentPosition.row),
				   (newPos.column - mRobot.mCurrentPosition.column))*180/PI);
      updateRobotPosition(newPos.row, newPos.column);

      // Remove path cell from grid and from stack
      mrGrid->removePathCell(newPos);
      mPathToDest.pop();

   }
   return validUpdate;
}



/*******************************************************************************
 * nextPosition
 *
 *      Fills the pointer sent as argument with the next position (row and column in
 *  respect to the grid) in the path. Returns true if a next position vector is found.
 *
 * @param  [out]  apPos
 *
 * @return knownNextPos
 *
 *******************************************************************************/
bool NavigationModel::nextPosition(Position *apPos)
{
   bool knownNextPos = false;

   if( !mPathToDest.empty() )
   {
      knownNextPos = true;
//      printf("nextPos is %i, %i\n",mPathToDest.top().row, mPathToDest.top().column);

      apPos->row = mPathToDest.top().row;
      apPos->column = mPathToDest.top().column;
   }
   else
   {
      if(mRobot.mCurrentPosition.row == mFinalPosition.row &&
         mRobot.mCurrentPosition.column == mFinalPosition.column)
      {
	 printf("Good job!\n");
      }
      else
      {
         printf("Path is not known, can't determine the next position.\n");
      }
   }
   return knownNextPos;
}



/*******************************************************************************
 * nextPositionVector
 *
 * Fills the position vector pointer sent with the vector in respect to the robot
 * reference frame. The position and orientation of the robot is updated.
 *
 * @param  [out]  aNextPosVec
 *
 * @return knownNextPosVector
 *
 *******************************************************************************/
bool NavigationModel::nextPositionVector(vector<float> *apNextPosVec)
{
   bool knownNextPosVector = false;
   Position nextPos;

   // Get the next position
   if( nextPosition(&nextPos) )
   {
      // Calculate the position vector in respect to the grid
      vector<float> posVecGrid;
      posVecGrid.push_back(nextPos.row - mRobot.mCurrentPosition.row);
      posVecGrid.push_back(nextPos.column - mRobot.mCurrentPosition.column);

      // Transform position vector to become in respect to the robot
      vector<float> tempVec;
      tempVec = transMatGtoR(posVecGrid);

      // Push back results in the pointed vector
      apNextPosVec->push_back(tempVec[0]*GRID_CM);
      apNextPosVec->push_back(tempVec[1]*GRID_CM);

      // Set the robot's new position and orientation
 //     updateRobotOrientation(atan2(posVecGrid[0], posVecGrid[1])*180/PI);
 //     updateRobotPosition(nextPos.row, nextPos.column);

      knownNextPosVector = true;
   }

   return knownNextPosVector;
}





/*******************************************************************************
 * calculatePathToDest
 *
 *      Finds the path to the selected destination using the A* algorithm private
 * function (see aStarAlgorithm()). Other algorithm can be used here.
 *
 *
 * @return mPathFound
 *
 *******************************************************************************/
bool NavigationModel::calculatePathToDest()
{
   bool mPathFound = false;
   
   if( mKnownDestination )
   {
      // A* algorithm
      if( aStarAlgorithm() ){
         mPathFound = true;
      }
   }
   else
   {
      printf("Error: No destination given.\n");
   }
   return mPathFound;
}




/*******************************************************************************
 * addObstacle
 *
 * ... to be completed ... for now adds an obstacle base of aAngle (for row) and
 * aDistance (for column).
 *
 * @param  [in]  aSensorID
 * @param  [in]  aAngle
 * @param  [in]  aDistance
 *
 * @return true
 *
 *******************************************************************************/
bool NavigationModel::addObstacle(vector<float> aSensorDistances)
{
   bool pathOK = true;
   bool recalculatePath = false;

   // Vector of obstacles to add
   vector<Position> newObstacles;

   // Initialize vector to model the 8 possibile direction (right, left, down, up and 4 diagonals)
//   vector<int> row8dir={0, 0, 1,-1, 1, 1,-1,-1};
//   vector<int> col8dir={1,-1, 0, 0, 1,-1, 1,-1};

   vector<int> row8dir={0, 0, 1,-1, 1, 1,-1,-1, 2,-2, 0, 0};
   vector<int> col8dir={1,-1, 0, 0, 1,-1, 1,-1, 0, 0, 2,-2};

   double angleFromX = ANGLE_BETWEEN_SENSORS*NUM_SENSORS;

   // Loop through the 5 sensors
   for( unsigned int sensorID = 0 ; sensorID < aSensorDistances.size() ; sensorID++, angleFromX-=ANGLE_BETWEEN_SENSORS)
   {
      // An obstacle was detected in the "observable zone" or too close to the robot
      if ( aSensorDistances[sensorID] != 0 )
      {
         vector<float> obstRframe;

	 // Obstacle is too close to the robot, set distance to minimum
         if( aSensorDistances[sensorID] == -1 )
         {
            obstRframe.push_back((ROBOT_RADIUS + MIN_OBS_ZONE)*cos(angleFromX*PI/180));  // X in respect to robot frame
            obstRframe.push_back((ROBOT_RADIUS + MIN_OBS_ZONE)*sin(angleFromX*PI/180));  // Y in respect to robot frame
	 }
	 // Obstacle is in observable zone so the sensor distance is used
         else
         {
            obstRframe.push_back((ROBOT_RADIUS + aSensorDistances[sensorID])*cos(angleFromX*PI/180));  // X in respect to robot frame
            obstRframe.push_back((ROBOT_RADIUS + aSensorDistances[sensorID])*sin(angleFromX*PI/180));  // Y in respect to robot frame
	 }

         printf("%.02f (anglefromX = %.02f), obst in R frame %.02f,%.02f\n",aSensorDistances[sensorID],angleFromX, obstRframe[0],obstRframe[1]);

         // Transformation to obtain vector in respect ot grid
         vector<float> obstGframe = transMatRtoG(obstRframe);

         Position obstaclePos;
         obstaclePos.row = round(obstGframe[0]/GRID_CM);
         obstaclePos.column = round(obstGframe[1]/GRID_CM);

         // Add obstacle in grid
         if( !(mrGrid->addObstacle(obstaclePos)) )
         {
            recalculatePath = true;
         }

         // Add bufferzone around obstacle
         for( unsigned int i = 0 ; i < row8dir.size() ; i++ )
         {
//            cout << "Obst at : " <<obstaclePos.row+row8dir[i] <<","<<obstaclePos.column+col8dir[i] <<endl;
            if( !(mrGrid->addObstacle(Position(obstaclePos.row+row8dir[i], obstaclePos.column+col8dir[i]))) )
            {
               recalculatePath = true;
            }
         }
      }
      else // aSensorDistances[sensorID] == 0
      {
	  // Ignore, no obstacte were detected
      }
   }


   // If the recalculatePath was set to true, the path is recalculated
   if( recalculatePath )
   {
      cout << "Recalculating path ... " <<endl;
      clearPath();
      pathOK = calculatePathToDest();
   }

   return pathOK;
}




/*******************************************************************************
 * print
 *
 *      Prints the destination selected as well as the current grid and the current
 * robot position and orientation. For debugging purposes.
 *
 *
 *******************************************************************************/
void NavigationModel::print()
{
   printf("-------------------------------------------------------NavigationModel-------------------------------------------------------\n");
   printf("The destination is set to cell (%i, %i)\n",mFinalPosition.row,mFinalPosition.column);
   mrGrid->printGrid();
   mRobot.print();
   printf("------------------------------------------------------------------------------------------------------------------------------\n");
}

/*******************************************************************************
 * print
 *
 *      Prints the destination selected as well as the current grid and the current
 * robot position and orientation. For debugging purposes.
 *
 *
 *******************************************************************************/
void NavigationModel::print(FILE * aFile)
{
   fprintf(aFile,"-------------------------------------------------------NavigationModel--------------------------------------------------------\n");
   fprintf(aFile,"The destination is set to cell (%i, %i)\n",mFinalPosition.row,mFinalPosition.column);
   mrGrid->printGrid(aFile);
   mRobot.print(aFile);
   fprintf(aFile,"------------------------------------------------------------------------------------------------------------------------------\n");
   fflush(aFile);
}



/*******************************************************************************
 * transMatGtoR
 *
 *      Implementation of a 2D transformation from the grid to the robot without
 * taking into account the position. We can send a vector in respect to the grid
 * and the function will return the vector coordinates in respect to the robot.
 *
 * @param  [in]  aVectorGrid
 *
 * @return vecRobot
 *
 *******************************************************************************/
vector<float> NavigationModel::transMatGtoR(vector<float> aVectorGrid)
{
   vector<float> vecRobot;

   double aThetaRad = mRobot.mOrientation*PI/180;

   vecRobot.push_back(aVectorGrid[0]*cos(aThetaRad) - aVectorGrid[1]*sin(aThetaRad));
   vecRobot.push_back(aVectorGrid[1]*cos(aThetaRad) + aVectorGrid[0]*sin(aThetaRad));

   return vecRobot;

}




/*******************************************************************************
 * transMatRtoG
 *
 *      Implementation of a 2D transformation from the grid to the robot with
 * the consideration of the translation of the robot.
 *
 * @param  [in]  aVectorRobot
 *
 * @return vecGrid
 *
 *******************************************************************************/
vector<float> NavigationModel::transMatRtoG(vector<float> aVectorRobot)
{
   vector<float> vecGrid;

   double aThetaRad = mRobot.mOrientation*PI/180;

   vecGrid.push_back(aVectorRobot[0]*cos(aThetaRad) + aVectorRobot[1]*sin(aThetaRad) + mRobot.mCurrentPosition.row*GRID_CM);
   vecGrid.push_back(-aVectorRobot[0]*sin(aThetaRad) + aVectorRobot[1]*cos(aThetaRad) + mRobot.mCurrentPosition.column*GRID_CM);

   return vecGrid;
}

/*******************************************************************************
 * aStarAlgorithm
 *
 *   This algorithm was modified from the following open source c++ code:
 * http://code.activestate.com/recipes/577457-a-star-shortest-path-algorithm/history/4/
 *
 *
 * @return true if a path was found
 *
 *******************************************************************************/
bool NavigationModel::aStarAlgorithm()
{

   static int closed_nodes_map[NUM_ROW][NUM_COL]={};
   static int open_nodes_map[NUM_ROW][NUM_COL]={};
   static int direction_map[NUM_ROW][NUM_COL]={};
   static int drow[dir] = {1, 1, 0, -1, -1, -1,  0,  1};
   static int dcol[dir] = {0, 1, 1,  1,  0, -1, -1, -1};


   static priority_queue<node> pq[2];
   static int pqi;
   static node* n0;
   static node* m0;
   static int i, j, x, y, xdx, ydy;
   static char c;

   pqi=0;

    // reset the node maps
    for(y=0;y<NUM_COL;y++)
    {
        for(x=0;x<NUM_ROW;x++)
        {
            closed_nodes_map[x][y]=0;
            open_nodes_map[x][y]=0;
        }
    }
    // create the start node and push into list of open nodes
    n0=new node(mRobot.mCurrentPosition.row, mRobot.mCurrentPosition.column, 0, 0);
    n0->updatePriority(mFinalPosition.row, mFinalPosition.column);
    pq[pqi].push(*n0);
    open_nodes_map[x][y]=n0->getPriority(); // mark it on the open nodes map

    // A* search
    while(!pq[pqi].empty())
    {
        // get the current node w/ the highest priority
        // from the list of open nodes
        n0=new node( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(),
                     pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

        x=n0->getxPos();
        y=n0->getyPos();

        pq[pqi].pop(); // remove the node from the open list
        open_nodes_map[x][y]=0;
        // mark it on the closed nodes map
        closed_nodes_map[x][y]=1;

        // quit searching when the goal state is reached
        //if((*n0).estimate(xFinish, yFinish) == 0)
        if(x==mFinalPosition.row && y==mFinalPosition.column)
        {
            // generate the path from finish to start
            // by following the directions
            string path="";
            while(!(x==mRobot.mCurrentPosition.row && y==mRobot.mCurrentPosition.column))
            {
		mPathToDest.push(Position(x,y));
		mrGrid->addPathCell(x,y);
                j=direction_map[x][y];
                c='0'+(j+dir/2)%dir;
                path=c+path;
                x+=drow[j];
                y+=dcol[j];
            }

            // garbage collection
            delete n0;
            // empty the leftover nodes
            while(!pq[pqi].empty()) pq[pqi].pop();
            return true;
        }

        // generate moves (child nodes) in all possible directions
        for(i=0;i<8;i++)
        {
            xdx=x+drow[i]; ydy=y+dcol[i];

            if(!(xdx<0 || xdx>NUM_ROW-1 || ydy<0 || ydy>NUM_COL-1 || mrGrid->isOccupied(xdx, ydy)
                || closed_nodes_map[xdx][ydy]==1))
            {
               // generate a child node
                m0=new node( xdx, ydy, n0->getLevel(),
                             n0->getPriority());
                m0->nextLevel(i);
                m0->updatePriority(mFinalPosition.row, mFinalPosition.column);

                // if it is not in the open list then add into that
                if(open_nodes_map[xdx][ydy]==0)
                {
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    pq[pqi].push(*m0);
                    // mark its parent node direction
                    direction_map[xdx][ydy]=(i+dir/2)%dir;
                }
                else if(open_nodes_map[xdx][ydy]>m0->getPriority())
                {
                    // update the priority info
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    // update the parent direction info
                    direction_map[xdx][ydy]=(i+dir/2)%dir;

                    // replace the node
                    // by emptying one pq to the other one
                    // except the node to be replaced will be ignored
                    // and the new node will be pushed in instead
                    while(!(pq[pqi].top().getxPos()==xdx &&
                           pq[pqi].top().getyPos()==ydy))
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pq[pqi].pop(); // remove the wanted node

                    // empty the larger size pq to the smaller one
                    if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
                    while(!pq[pqi].empty())
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pqi=1-pqi;
                    pq[pqi].push(*m0); // add the better node instead
                }
                else delete m0; // garbage collection
            }
        }
        delete n0; // garbage collection
    }
    return false; // no route found
}

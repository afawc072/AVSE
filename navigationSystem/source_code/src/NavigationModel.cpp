/*******************************************************************************
*
* PROJET: AUTONOMOUS VEHICULE IN A STRUCTURED ENVIRONMENT
*
* SECTION: Navigation System
*
* AUTHORS: Jean-Sebastien Fiset and Alexandre Fawcett
*
* DESCRIPTION:
*
*	Navigation Model
*
* NOTES:
*
*
*
********************************************************************************/


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

   if(aDestinationID >=0 && aDestinationID < (int) mrGrid->getDestPositions().size())
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

   if(mRobot.mCurrentPosition.row == mFinalPosition.row &&
      mRobot.mCurrentPosition.column == mFinalPosition.column)
   {
      destinationReached = true;
   }

   return destinationReached;
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
 * @param  [in]  aCamServoAngle
 *
 * @return validLocalization
 *
 *******************************************************************************/
bool NavigationModel::localizeRobotInGrid(int tagID, double xdist_cm, double ydist_cm, double camServoAngle)
{
   bool validLocalization = false;

   Position tagPos = mrGrid->getTagPositions()[tagID];

   vector<int> row4dir={0,0,1,-1};
   vector<int> col4dir={1,-1,0,0};

   unsigned int i;
   for(i = 0 ; i <row4dir.size() ; i++)
   {
      if(mrGrid->isBuffer(tagPos.row + row4dir[i], tagPos.column + col4dir[i]))
      {
         break;
      }
   }
   cout << row4dir[i] <<","<< col4dir[i]<<endl;

   switch(i)
   {
      case 0:

      break;

      case 1:

      break;

      case 2:

      break;

      case 3:

      break;

      default:
      break;
   }

   updateRobotPosition(4,4);

   // Calculate and set the robot's orientation in respect to the grid
   double robotOrientation = camServoAngle - ((atan2(ydist_cm,xdist_cm)*180/PI));
   robotOrientation = 0;
   updateRobotOrientation(robotOrientation);

   return validLocalization;
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

   if(!mPathToDest.empty())
   {
      knownNextPos = true;
      printf("nextPos is %i, %i\n",mPathToDest.top().row, mPathToDest.top().column); 
      apPos->row = mPathToDest.top().row;
      apPos->column = mPathToDest.top().column;
      mPathToDest.pop();
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
 *	
 *
 * @param  [out]  aNextPosVec
 *
 * @return knownNextPosVector
 *
 *******************************************************************************/
bool NavigationModel::nextPositionVector(vector<float> *aNextPosVec)
{
   bool knownNextPosVector = false;
   Position nextPos;
   if(nextPosition(&nextPos))
   {
      vector<float> posVecGrid;
      posVecGrid.push_back(nextPos.row - mRobot.mCurrentPosition.row);
      posVecGrid.push_back(nextPos.column - mRobot.mCurrentPosition.column);

      vector<float> tempVec;
      tempVec = transMatGtoR(posVecGrid);
      aNextPosVec->push_back(tempVec[0]*GRID_CM);
      aNextPosVec->push_back(tempVec[1]*GRID_CM);

      // Set the robot's new position and orientation
      updateRobotOrientation(atan2(posVecGrid[0], posVecGrid[1])*180/PI);
      updateRobotPosition(nextPos.row, nextPos.column);

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
   if(mKnownDestination)
   {
      // A* algorithm
      if(aStarAlgorithm()){
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
 *
 *
 * @param  [in]  aSensorID
 * @param  [in]  aAngle
 * @param  [in]  aDistance
 *
 * @return true
 *
 *******************************************************************************/
bool NavigationModel::addObstacle(int aSensorID, float aAngle, float aDistance)
{
   printf("Sensor #%i at %.2f deg and %.2f cm\n",aSensorID,aAngle,aDistance);

   vector<Position> newObstacles;


   // Caculation of obstacle cells not yet implemented
   newObstacles.push_back(Position(aAngle,aDistance)); // TEMPORARY, FOR TESTING PURPOSES


   bool recalculatePath = false;

   for(unsigned int i = 0; i < newObstacles.size(); i++)
   {
      if( !(mrGrid->addObstacle(newObstacles[i])) )
      {
         while(!mPathToDest.empty())
	 {
	    mrGrid->removePathCell(mPathToDest.top());
	    mPathToDest.pop();
         }
	 recalculatePath = true;
      }
   }

   if(recalculatePath)
   {
      calculatePathToDest();
   }

   return true;
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
   printf("-------------------NavigationModel-------------------\n");
   printf("The destination is set to cell (%i, %i)\n",mFinalPosition.row,mFinalPosition.column);
   mrGrid->printGrid();
   mRobot.print();
   printf("-----------------------------------------------------\n");
}




/******************************************************************************* 
 * transMatGtoR
 *
 *      Implementation of a 2D transformation from the grid to the robot without
 * taking into account the position. We can send a vector in respect to the grid 
 * and the function will return the vector coordinates in respect to the robot.
 *
 * @param  [out]  aVectorGrid
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

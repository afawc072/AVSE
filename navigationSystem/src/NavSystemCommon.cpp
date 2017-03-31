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
*	 Navigation System common struct and class functions implementation
*
* NOTES:
*
*
*
********************************************************************************/
/** @file NavSystemCommon.cpp*/

#include "NavSystemCommon.h"


/*******************************************************************************
 * Position
 *
 *    Default constructor
 *
 *******************************************************************************/
Position::Position()
{
}



/*******************************************************************************
 * Position
 *
 *      Constructor setting a row and col for the struct
 *
 * @param  [in]  aRow
 * @param  [in]  aCol
 *
 *******************************************************************************/
Position::Position(int aRow, int aCol)
{
  row=aRow;
  column=aCol;
}



/*******************************************************************************
 * Robot
 *
 *    Default constructor
 *
 *******************************************************************************/
Robot::Robot()
{
}



/*******************************************************************************
 * updatePosition
 *
 *    Updates the row and column index of the robot
 *
 *  @param  [in]  aNewRow
 *  @param  [in]  aNewCol
 *
 *******************************************************************************/
void Robot::updatePosition(int aNewRow, int aNewCol)
{
   mCurrentPosition.row=aNewRow;
   mCurrentPosition.column=aNewCol;
}



/*******************************************************************************
 * print
 *
 *    Prints the current position and orienation of the robot
 *
 *******************************************************************************/
void Robot::print()
{
   printf("-------------------------------------------------------Robot------------------------------------------------------------------\n");
   printf("Position: (%i, %i) \n", mCurrentPosition.row, mCurrentPosition.column);
   printf("Orientation: %f\n", mOrientation);

}



/*******************************************************************************
 * print
 *
 *    Prints the current position and orienation of the robot
 *
 *******************************************************************************/
void Robot::print(FILE * aFile)
{
   fprintf(aFile,"-------------------------------------------------------Robot------------------------------------------------------------------\n");
   fprintf(aFile,"Position: (%i, %i) \n", mCurrentPosition.row, mCurrentPosition.column);
   fprintf(aFile,"Orientation: %f\n", mOrientation);
   fflush(aFile);
}



node::node(int xp, int yp, int aLevel, int aPriority)
{
   xPos = xp;
   yPos = yp;
   level = aLevel;
   priority = aPriority;
}

int node::getxPos() const {return xPos;}

int node::getyPos() const {return yPos;}

int node::getLevel() const {return level;}

int node::getPriority() const {return priority;}

void node::updatePriority(const int & aNewX, const int & aNewY)
{
   priority = level + estimate(aNewX, aNewY)*10;
}

void node::nextLevel(const int & aDirection)
{
   level += (dir==8?(aDirection%2==0?10:14):10);
}

const int & node::estimate(const int & xDest, const int & yDest) const
{
   static int xd, yd, d;
   xd=xDest-xPos;
   yd=yDest-yPos;

   // Euclidian Distance
   d=static_cast<int>(sqrt(xd*xd+yd*yd));

   // Manhattan distance
   //d=abs(xd)+abs(yd);

   // Chebyshev distance
   //d=max(abs(xd), abs(yd));

   return(d);
}

bool operator<(const node & first, const node& second)
{
   return first.getPriority() > second.getPriority();
}

#include "NavSystemCommon.h"


Position::Position()
{
}


Position::Position(int aRow, int aCol)
{
  row=aRow;
  column=aCol;
}

Robot::Robot()
{
}


void Robot::updatePosition(int aNewRow, int aNewCol)
{
   mCurrentPosition.row=aNewRow;
   mCurrentPosition.column=aNewCol;
}


void Robot::print()
{
   printf("---------------------Robot-----------------\n");
   printf("Position: (%i, %i) \n", mCurrentPosition.row, mCurrentPosition.column);
   printf("Orientation: %f\n", mOrientation);

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

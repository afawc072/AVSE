#ifndef NAVSYSTEMCOMMON_H
#define NAVSYSTEMCOMMON_H

#include <fstream>
#include <vector>
#include <string>
#include <iostream>
#include <math.h>
#include <stack>

#include <iomanip>
#include <queue>
#include <ctime>
#include <stdio.h>
#include <stdlib.h>


#define PI 3.141592654

// IMPORTANT SETTINGS
static const int GRID_CM = 10;
static const int NUM_COL = 50;
static const int NUM_ROW = 30;

static const int dir = 8; //Number of possibles directions for the robot (4 or 8) 


using namespace std;


// Representing a position in the Grid
struct Position
{
   Position();
   Position(int aRow, int aCol);

   int row;
   int column;
};



// Define all the possible cell states
typedef enum
{
   W, // Wall
   T, // Tag
   O, // Obstacle
   F, // Free
   P, // Path
   D, // Destination
   B, // Buffer
} cstate;

// For printing purposes
static const vector<char> cstate_names={'W','T','O','F','P','D','B'};


/*
static const cstate INITIAL_GRID[10][10] = {
					   {W,W,W,W,W,W,W,W,W,W},
					   {W,F,F,F,F,F,F,F,F,W},
                                           {W,F,F,F,F,F,F,F,F,W},
                                           {W,F,F,F,W,W,F,F,F,W},
                                           {W,F,F,F,W,W,F,F,F,W},
                                           {W,W,W,W,W,W,F,F,F,W},
                                           {W,W,W,W,W,W,F,F,F,W},
                                           {W,F,F,F,F,F,F,F,F,W},
                                           {W,F,F,F,F,F,F,F,F,W},
                                           {W,W,W,W,W,W,W,W,W,W}
					   };
*/



// Struct to represent the robot. Contains it position and orientation
struct Robot
{
   Robot();

   double mOrientation;
   Position mCurrentPosition;

   void updatePosition(int aNewRow, int aNewCol);

   void print();
};



// FOR A* ALGORITHM
class node
{
   public:
      node(int xp, int yp, int aLevel, int aPriority);

      int getxPos() const;

      int getyPos() const;

      int getLevel() const;

      int getPriority() const;

      void updatePriority(const int & aNewX, const int & aNewY);

      void nextLevel(const int & aDirection);

      const int & estimate(const int & xDest, const int & yDest) const;

   private:

      int xPos;
      int yPos;

      int level;

      int priority;

};


bool operator<(const node & first, const node & second);




#endif /*NAVSYSTEMCOMMON_H*/

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
*       Navigation System Common Header File
*
* NOTES:
*
*	Includes the necessary modules, defines the constant variable used in this
* section of the project as well as the node class (for A* algorithm). The file also
* defines the following struct:
*
*	  - Position : contain a row and column index
* 	- Robot : contain a Position and an orientation
*
*
********************************************************************************/
/** @file NavSystemCommon.h */


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
static const int NUM_ROW = 25;

static const int ROBOT_RADIUS = 15;
static const int CAM_RCENTER_DIST = 6;
static const int ANGLE_BETWEEN_SENSORS = 30;
static const int NUM_SENSORS = 5;
static const int MIN_OBS_ZONE = 15;

static const int dir = 8; //Number of possibles directions for the robot (4 or 8)


using namespace std;


/*! \enum cstate
*
*   \brief Define the possible cell state for the grid
*
* Contain the elements:
*
*   W, representing a wall
*   T, representing a tag
*   O, for an obstacle cell
*   F, for a free cell
*   P, for a path cell
*   D, for a destination cell
*   B, for a buffer cell
*
*******************************************************************************/
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



/*! \struct Position
 *
 * \brief Represents a position in the grid
 *
 *
 *******************************************************************************/
struct Position
{
   Position();
   Position(int aRow, int aCol);

   int row;
   int column;
};



/*! \struct Robot
 *
 * \brief Represents a robot and contains a Position and an orientation in respect
 * to the grid axis.
 *
 *
 *******************************************************************************/
struct Robot
{
   Robot();

   double mOrientation;
   Position mCurrentPosition;

   void updatePosition(int aNewRow, int aNewCol);

   void print();
   void print(FILE* aFile);

};



/*! \brief Defines a node in the grid for the A* algorithm
 *
 *  This class was taken from the open source c++ code :
 *  http://code.activestate.com/recipes/577457-a-star-shortest-path-algorithm/history/4/
 *
 *
 *******************************************************************************/
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

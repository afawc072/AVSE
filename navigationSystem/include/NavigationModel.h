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
*	Navigation Model Header File
*
* NOTES:
*
*
*
********************************************************************************/
/** @file NavigationModel.h*/

#include "Grid.h"
#include "NavSystemCommon.h"


/*! \brief Defines the navigation model of the AVSE
 *
 *  This class contains a grid and a robot and is responsible for the path finding
 * part of the project. By setting the final desired destination and localizing the
 * robot within the grid, the quickest path can be calculated. If a obstacle is
 * detected along the way, it can be added to the occupancy grid and a recalculation
 * of the path occurs if the obstacles is in the path of the robot. Every time a
 * cell is reached, the next position vector can be sent.
 *
 *
 *******************************************************************************/
class NavigationModel{

    public:

      // Default constructor
      NavigationModel();

      // Constructor when a pointer to a Grid is sent
      NavigationModel(Grid *apGrid);

      // Sets the desired final destination
      bool setDestination(int aDestinationID);

      // Returns true if the robot position match the final destination position
      bool destinationIsReached();

      // Clears the path stack and change the state of the path cell from P to F
      void clearPath();


      // Updates robot orientation
      void updateRobotOrientation(double aOrientation);

      // Updates the robot position
      void updateRobotPosition(int aNewRow, int aNewCol);


      // Knowing the tagID and x,z distance from this tag to the robot, we localize the robot in the grid
      bool localizeRobotInGrid(int aTagID, double aXdist_cm, double aZdist_cm, double aCamMarkerAngle, double aCamServoAngle);


      // Fills the position pointer with the next position in the path
      bool nextPosition(Position *apPos);

      // Based on the robot current position and orientation, calculate and fill vector pointer and updates the robot position
      bool nextPositionVector(vector<float> *apNextPosVec);

      // Calculate and set the path cell guiding the robot to the final destination set
      bool calculatePathToDest();

      // (To be completed)
      bool addObstacle(vector<float> aSensorDistances);

      // Prints the object current state
      void print();


    private:

      // Pointer to a Grid object
      Grid *mrGrid;

      // Robot object (containing position and orientation)
      Robot mRobot;

      // Stack containing the position toward the destination
      stack<Position> mPathToDest;

      // Final destination position
      Position mFinalPosition;

      // Flag to be set to true when destination is known
      bool mKnownDestination;


      // Private function that transform a position vector in respect to the grid to
      // a position vector in respect to the robot
      vector<float> transMatGtoR(vector<float> aVectorGrid);

      // Implementation of the A* algorithm
      bool aStarAlgorithm();
};


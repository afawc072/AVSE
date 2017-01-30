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
	NavigationModel();

	NavigationModel(Grid *apGrid);


        bool setDestination(int aDestinationID);

	bool destinationIsReached();

	void clearPath();

        bool localizeRobotInGrid(int aTagID, double aXdist_cm, double aZdist_cm, double aCamServoAngle);

	// Update robot information
	void updateRobotOrientation(double aOrientation);

	void updateRobotPosition(int newRow, int newCol);


	bool nextPosition(Position *apPos);

	bool nextPositionVector(vector<float> *aNextPosVec);

	bool calculatePathToDest();


	bool addObstacle(int aSensorID, float aAngle, float aDistance);


	void print();

    private:

	Grid *mrGrid;

	Robot mRobot;

	stack<Position> mPathToDest;

	Position mFinalPosition;

	bool mKnownDestination;


	// Private function that transform a position vector in respect to the grid to
	// a position vector in respect to the robot
	vector<float> transMatGtoR(vector<float> aVectorGrid);

	// Implementation of the A* algorithm
        bool aStarAlgorithm();
};


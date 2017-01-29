#include "Grid.h"
#include "NavSystemCommon.h"

class NavigationModel{

    public:
	NavigationModel();

	NavigationModel(Grid *apGrid);


        bool setDestination(int aDestinationID);

	bool destinationIsReached();


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


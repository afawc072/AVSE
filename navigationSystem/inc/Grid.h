#ifndef GRID
#define GRID

#include "NavSystemCommon.h"


/*! \brief Defines a simplified occupancy grid with tags and destinations
 *
 *	This class contains a matrix that is filled according to a file where the
 *  different cell state are:
 *
 * -W represents a wall cell
 * -F represents a free cell
 * -T represents a tag cell
 * -D represents a destination cell
 * -B represents a buffer zone cell
 * -O represents a obstacle cell
 * -P represents a path cell -> these are changed during the execution
 *
 *  	Each cell represent a square in the real world of a size specified in the
 * NavSystemCommon header file. When reading the Grid.txt file, the tag ID and
 * destination ID are determined (starting at 0) when reading row by row from left
 * to right. The class also store a vector containing the positions of the tags and
 * the position of the destination.
 *
 *******************************************************************************/
class Grid
{
    public:

	Grid();

	void resetGrid();

	vector<Position> getDestPositions();

	vector<Position> getTagPositions();


        bool isBuffer(int aRow, int aCol);

	bool isOccupied(int aRow, int aCol);


	bool addTag(Position aNewTag);

	bool addObstacle(Position aNewObstacle);

	bool addPathCell(int row, int col);

	bool removePathCell(Position cellToRemove);


	void printGrid();

    private:

	cstate mGrid[NUM_ROW][NUM_COL];

	vector<Position> mDestPositions;

	vector<Position> mTagPositions;


	void setUpTags();

        void readGridFromFile(string filename);
};
#endif /*GRID*/

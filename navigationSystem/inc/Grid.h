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
*	 Grid Class Header File
*
* NOTES:
*
*
*
********************************************************************************/
/** @file Grid.h*/

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

	// Default constructor
	Grid();

	// Constructor taking a file path in argument
	Grid(string aGridFilePath);

  	// Resets the Grid to the grid specify in the file
	void resetGrid();

  	// Returns the destination positions vector
	vector<Position> getDestPositions();

	// Returns the destination positions vector
	vector<Position> getTagPositions();


	// Sets the file path for the grid that will be used
        void setGridFilePath(string aFilePath);

  	// Returns true if mGrid[aRow][aCol] is a buffer cell
        bool isBuffer(int aRow, int aCol);

  	// Returns true if cell is occupied (B,W or T) or out of bounds
	bool isOccupied(int aRow, int aCol);

  	// Changes the state of the cell if free or in path. If in path, recalculate path from current position
	bool addObstacle(Position aNewObstacle);

  	// Changes the state of the cell to a path cell
	bool addPathCell(int aRow, int aCol);

  	// Change the state of a path cell specified to free and return true if this cell was a path cell.
	bool removePathCell(Position aCellToRemove);

  	// Display the current grid
	void printGrid();


    private:

  	// Matrix of cell states
	cstate mGrid[NUM_ROW][NUM_COL];

	// File path pointing to the matrix file
	string mGridFilePath;

  	// Vector containing the position of each destination in the order of their DestID
	vector<Position> mDestPositions;

  	// Vector containing the position of each tags in the order of their tagID
	vector<Position> mTagPositions;

        // Reads the grid from the filename specified
        void readGridFromFile(string aFilename);
};

#endif /*GRID*/

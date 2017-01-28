#ifndef GRID
#define GRID

#include "NavSystemCommon.h"


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

#include "Grid.h"



Grid::Grid()
{
   resetGrid();
}


void Grid::resetGrid() {
   readGridFromFile("Grid.txt");
/*
   for(int rows=0; rows< NUM_ROW; rows++){
      for(int cols=0; cols< NUM_COL; cols++){
         mGrid[rows][cols]=INITIAL_GRID[rows][cols];
      }
   }
   setUpTags();*/
}


vector<Position> Grid::getDestPositions()
{
   return mDestPositions;
}

vector<Position> Grid::getTagPositions()
{
   return mTagPositions;
}


bool Grid::isBuffer(int aRow, int aCol)
{
   if(aRow>=0 && aRow < NUM_ROW && aCol>=0 && aCol<NUM_COL)
   {
      return mGrid[aRow][aCol] == B;
   }
   return false;
}

bool Grid::isOccupied(int aRow, int aCol)
{
   return !(mGrid[aRow][aCol] == F || mGrid[aRow][aCol] == P || mGrid[aRow][aCol] == D);
}



// Not used
bool Grid::addTag(Position aNewTag){

   bool validTag=true;
   int row = aNewTag.row;
   int col = aNewTag.column;

   if(row>=0 && row < NUM_ROW && col>=0 && col<NUM_COL)
   {
      if(mGrid[row][col]==W)
      {
         mGrid[row][col]=T;
      }
      else
      {
         printf("Cell (%i,%i) is not a wall\n",row,col);
         validTag=false;
      }
   }
   else
   {
      validTag=false;
   }
   return validTag;
}




bool Grid::addObstacle(Position aNewObstacle){

   bool validObst=true;
   int row = aNewObstacle.row;
   int col = aNewObstacle.column;

   if(row>=0 && row < NUM_ROW && col>=0 && col<NUM_COL)
   {
      if(mGrid[row][col]==P)
      {
         validObst=false;
      }
      mGrid[row][col]=O;
   }
   else
   {
      //Ignore since obstacle index is not within the grid
   }

   return validObst;
}



bool Grid::addPathCell(int row, int col)
{
   bool validPathCell = false;

   if(row>=0 && row < NUM_ROW && col>=0 && col<NUM_COL)
   {
      mGrid[row][col]=P;
   }

   return validPathCell;
}




bool Grid::removePathCell(Position cellToRemove)
{
   bool pathCell = false;
   if(mGrid[cellToRemove.row][cellToRemove.column] == P)
   {
      mGrid[cellToRemove.row][cellToRemove.column] = F;
      pathCell = true;
   }
   return pathCell;
}

void Grid::printGrid(){
   printf("\n----------------------GRID-------------------------\n");
   for(unsigned int rows=0; rows<NUM_ROW; rows++){
      for(unsigned int cols=0; cols<NUM_COL; cols++){
          printf("%c ",cstate_names[mGrid[rows][cols]]);
      }
	printf("\n");
   }
   printf("------------------------------------------------------\n");
}


/*
void Grid::setUpTags()
{
   for(unsigned int i = 0; i < TAG_POSITION.size() ; i++)
   {
      mGrid[TAG_POSITION[i].row][TAG_POSITION[i].column] = T;
   }
}
*/


void Grid::readGridFromFile(string filename)
{
   ifstream grid (filename);
   bool warning = false;
   int lines_count = 0;
   int row=0;
   string line;
   while(getline(grid,line))
   {
      lines_count++;
      int col=0;
      for(unsigned int i = 0 ; i<line.length() ; i++)
      {
	 if(col == NUM_COL){warning=true; break;}
         switch(line[i])
	 {
	    case 'F':
		mGrid[row][col]=F;
		col++;
		break;
	    case 'W':
		mGrid[row][col]=W;
		col++;
		break;
	    case 'T':
		mGrid[row][col]=T;
		mTagPositions.push_back(Position(row,col));
		col++;
		break;
	    case 'D':
		mGrid[row][col]=D;
		mDestPositions.push_back(Position(row,col));
		col++;
		break;
	    case 'B':
		mGrid[row][col]=B;
		col++;
	    default:
		// Do nothing
		break;
         }
      }
      if(row++ == NUM_ROW) break;
   }

   if(lines_count != NUM_ROW) printf("The number of lines expected in Grid.txt is 30 but %i was found (to be fixed)",lines_count);
   if(warning) printf("The number of columns expected in Grid.txt is 50 but more was found in a certain line(to be fixed)");

}

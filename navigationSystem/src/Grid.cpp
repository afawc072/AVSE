
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
*	  Grid Class Function Implementation
*
* NOTES:
*
*
*
********************************************************************************/
/** @file Grid.cpp*/

#include "Grid.h"



/*******************************************************************************
 * Grid
 *
 * 	Default constructor
 *
 *******************************************************************************/
Grid::Grid()
{
}



/*******************************************************************************
 * Grid
 *
 *      Constructor taking a filepath as argument to set the grid to use
 *
 *  @param  [in]  aGridFilePath
 *
 *******************************************************************************/
Grid::Grid(string aGridFilePath)
{
   mGridFilePath = aGridFilePath;
   resetGrid();
}




/*******************************************************************************
 * resetGrid
 *
 * 	Resets the Grid based on the Grid.txt file
 *
 *******************************************************************************/
void Grid::resetGrid()
{
   readGridFromFile(mGridFilePath);
}



/*******************************************************************************
 * setGridFilePath
 *
 *  Sets the file path that will be used to reset the grird
 *
 * @param [in]  aFilePath
 *
 *******************************************************************************/
void Grid::setGridFilePath(string aFilePath)
{
   mGridFilePath = aFilePath;
}



/*******************************************************************************
 * getDestPositions
 *
 *  Returns the vector containing the destination positions
 *
 * @return    mDestPositions
 *
 *******************************************************************************/
vector<Position> Grid::getDestPositions()
{
   return mDestPositions;
}



/*******************************************************************************
 * getTagPositions
 *
 *  Returns the vector containing the tag positions
 *
 * @return    mTagPositions
 *
 *******************************************************************************/
vector<Position> Grid::getTagPositions()
{
   return mTagPositions;
}



/*******************************************************************************
 * isBuffer
 *
 *  Returns true if the cell at the row and column index specified is a Buffer cell
 *
 *  @param [in]   aRow
 *  @param [in]   aCol
 *
 *  @return    fBufferCell
 *
 *******************************************************************************/
bool Grid::isBuffer(int aRow, int aCol)
{
   bool fBufferCell = false;

   // Check if row and column specified a cell inside the grid
   if( aRow>=0 && aRow < NUM_ROW && aCol>=0 && aCol<NUM_COL )
   {
       // Check if the state of the cell is B (for buffer)
       if( mGrid[aRow][aCol] == B )
       {
          fBufferCell = true;
       }
   }
   return fBufferCell;
}



/*******************************************************************************
 * isOccupied
 *
 *  Returns true if the cell specified is either in state "free", "path" or "destination"
 *  **Also returns true is row and column index are not inside the grid
 *
 *  @param [in]   aRow
 *  @param [in]   aCol
 *
 *  @return    fOccupied
 *
 *******************************************************************************/
bool Grid::isOccupied(int aRow, int aCol)
{
   bool fOccupied = true;

   // Check if row and column specified a cell inside the grid
   if( aRow>=0 && aRow < NUM_ROW && aCol>=0 && aCol<NUM_COL )
   {
      fOccupied = !(mGrid[aRow][aCol] == F || mGrid[aRow][aCol] == P || mGrid[aRow][aCol] == D);
   }

  return fOccupied;

}



bool Grid::isNextToFree( int aRow, int aCol )
{
   bool freeCellFound = false;
   vector<int> row4dir={0, 0, 1,-1};
   vector<int> col4dir={1,-1, 0, 0};

   for( unsigned int i = 0; i<row4dir.size() ; i++ )
   {
      if( !isOccupied(aRow+row4dir[i], aCol+col4dir[i]) )
      {
         freeCellFound = true;
         break;
      }
   }

   return freeCellFound;
}

/*******************************************************************************
 * addObstacle
 *
 *  Change the state of the cell specified if it is inside the grid and is free.
 *    - If the cell is not inside the grid or is already occupied, we ignore it.
 *    - If the cell is a free cell, we change the state to occupied and return true.
 *    - If the cell is a path cell, we change the state to occupied and return false.
 *
 *  @param [in]   aNewObstacle
 *
 *  @return    fNotInPath
 *
 *******************************************************************************/
bool Grid::addObstacle(Position aNewObstacle){

   bool fNotInPath = true;

   // Get the row and column
   int row = aNewObstacle.row;
   int col = aNewObstacle.column;

   // Check if cell is inside the grid and not occupied
   if( !isOccupied(row,col) )
   {
      if( mGrid[row][col]==P )
      {
         fNotInPath=false;
      }
      // Change the state to occupied
      mGrid[row][col]=O;
   }
   else
   {
      //Ignore since obstacle index is not within the grid or already occupied
   }

   return fNotInPath;
}



/*******************************************************************************
 * addPathCell
 *
 *  Change the state of the cell specified if it is inside the grid and is not occupied
 *    - If the cell is not inside the grid or occupied, we ignore and return false
 *    - If the cell is inside the gird and not occupied, we change the state to path
 *      and return true.
 *
 *  @param [in]   aRow
 *  @param [in]   aCol
 *
 *  @return    fNotInPath
 *
 *******************************************************************************/
bool Grid::addPathCell(int aRow, int aCol)
{
   bool fValidPathCell = false;

   if( !isOccupied(aRow, aCol) )
   {
      mGrid[aRow][aCol]=P;
      fValidPathCell = true;
   }

   return fValidPathCell;
}



/*******************************************************************************
 * removePathCell
 *
 *  Change the state of a path cell specified to free and return true if this cell was
 *  a path cell.
 *
 *  @param [in]   aCellToRemove
 *
 *  @return    fPathCell
 *
 *******************************************************************************/
bool Grid::removePathCell(Position aCellToRemove)
{
   bool fPathCell = false;

   if( mGrid[aCellToRemove.row][aCellToRemove.column] == P )
   {
      mGrid[aCellToRemove.row][aCellToRemove.column] = F;
      fPathCell = true;
   }
   return fPathCell;
}



/*******************************************************************************
 * printGrid
 *
 *  Prints the content of the Grid (for testing purposes)
 *
 *
 *******************************************************************************/
void Grid::printGrid()
{
   printf("\n----------------------------------------------------------GRID--------------------------------------------------------------\n");
   for(unsigned int rows=0; rows<NUM_ROW; rows++)
   {
      for(unsigned int cols=0; cols<NUM_COL; cols++)
      {
          printf("%c ",cstate_names[mGrid[rows][cols]]);
      }
      printf("\n");
   }
   printf("------------------------------------------------------------------------------------------------------------------------------\n");
}



/*******************************************************************************
 * printGrid
 *
 *  Prints the content of the Grid (for testing purposes)
 *
 *
 *******************************************************************************/
void Grid::printGrid(FILE * aFile)
{
   fprintf(aFile,"\n-------------------------------------------------------GRID-----------------------------------------------------------------\n");
   for(unsigned int rows=0; rows<NUM_ROW; rows++)
   {
      for(unsigned int cols=0; cols<NUM_COL; cols++)
      {
          fprintf(aFile,"%c ",cstate_names[mGrid[rows][cols]]);
      }
      fprintf(aFile,"\n");
   }
   fprintf(aFile,"------------------------------------------------------------------------------------------------------------------------------\n");
   fflush(aFile);
}



/*******************************************************************************
 * readGridFromFile
 *
 *  Reads the file specified to build the Grid stored in the object.
 *
 *  @param [in]   aFilename
 *
 *******************************************************************************/
void Grid::readGridFromFile( string aFilename )
{
   // Open the file specified
   ifstream grid (aFilename);
   bool warning = false;
   int lines_count = 0;
   int row=0;
   string line;

   // Go through all the lines in the file
   while( getline(grid,line) )
   {
      lines_count++;
      int col=0;

      // Go through the columns in the current line
      for( unsigned int i = 0 ; i<line.length() ; i++ )
      {
	 // If we see that a line contains more column than specified, we ignore and throw a warning
	 if( col == NUM_COL ){warning=true; break;}

         // Assign the cell state according to the character found
	 switch( line[i] )
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

      // If there is more row in the file than specified, stop analyzing the file
      if(row++ == NUM_ROW) break;
   }

   // Print a message if the number of row or the number of column in the file didn't match
   if(lines_count != NUM_ROW) printf("The number of lines expected in Grid.txt is 30 but %i was found (to be fixed)",lines_count);
   if(warning) printf("The number of columns expected in Grid.txt is 50 but more was found in a certain line(to be fixed)");

}

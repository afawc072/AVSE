***VERIFY AND CORRECT EQUATION TO FIND THE INITIAL ORIENTATION OF THE ROBOT***

This section of the AVSE contains the c++ model to simulate the Navigation system. All the variables parameters (such as the Grid size in cm or the number of rows and columns) are defined in the source_code/inc/NavSystemCommon.h file. The Grid.txt contains the working grid with the following cell states:
- F for free cell
- W for a wall cell
- T for a tag cell
- D for a destination cell
- B for a buffer cell (the robot can't be centered in those cells)

For the Tags and Destination IDs, these are defined when reading the Grid.txt file and the ID are starting at 0 and incrementing while going through each row, from left to right.

(To be fixed) If the length and width of the grid are modified, the number of row and column also need to be modified in the source_code/inc/NavSystemCommon.h file as well.

To run the simulation, run the NavSystem executable (type ./bin/NavSystem from this directory) and input the desired destination ID

Here are the file contained in this section: 

├── bin
│   └── NavSystem
├── Grid.txt
├── include
│   ├── Grid.h
│   ├── NavigationModel.h
│   └── NavSystemCommon.h
├── Makefile
├── obj
│   ├── Grid.d
│   ├── Grid.o
│   ├── main.d
│   ├── main.o
│   ├── Navigation.d
│   ├── NavigationModel.d
│   ├── NavigationModel.o
│   ├── NavSystemCommon.d
│   ├── NavSystemCommon.o
│   └── Robot.d
├── README.md
└── src
    ├── Grid.cpp
    ├── main.cpp
    ├── NavigationModel.cpp
    └── NavSystemCommon.cpp



This directory is aimed towards the computer vision application of this system. Please read
the SETUP.txt in order to setup the system appropriately(librairies, modules, etc.).

TO-DO List:

-Makefile for all source code but to be easily modular for integration elsewhere.
-Write aruco detection code - look at detect_markers
-Calibration running and modification to find detection params and camera parameters.
-Find camera pose(Matrix Transformation, etc.)
-Extract Data from tag detection:pid, x,y,z, angle
-Test Distances and angle from camera

DONE:

-Opencv and contribution installed

MODIFICATIONS

-Abandonned ARTOOLKIT(processing power and cameraread problem)-> ARUCO module for opencv


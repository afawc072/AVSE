/*******************************************************************************
*
* PROJET: AUTONOMOUS VEHICULE IN A STRUCTURED ENVIRONMENT
*
* SECTION: ComputerVision
*
* AUTHORS: Alexandre Fawcett
*
* DESCRIPTION:
*
*	Protocol header file
*
* NOTES:
*
*
*
********************************************************************************/
#ifndef COMPUTERVISION
#define COMPUTERVISION

#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <ctime>

using namespace std;
using namespace cv;

// IMPORTANT SETTINGS
//static const int SLEEP_S = 1;//Short Sleep(ms);
//static const int SLEEP_M = 1.5;//Medium Sleep(ms);
//static const int SLEEP_L = 2;//Long Sleep(ms);

static const int TIME_DELAY = 10;

static const aruco::PREDEFINED_DICTIONARY_NAME DICT_ID = aruco::DICT_ARUCO_ORIGINAL;
static const float MARKER_LENGTH = 0.1;
//static const string CAM_PARAM="data/camparam.yml";

//PROTOCOL DICTIONNARY


class computervision
{

/*
It is to be noted that the cartesian parameters(x,z) are the position of the camera
in the marker's frame.
*/
public:
	computervision();
	computervision(string camParam);
	bool detectTag(int aMaxTagID, int &tagID, double &xCam, double &zCam, double &angle);



private:

	string mCamParam;

	bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs);

};

#endif /*COMPUTERVISION*/

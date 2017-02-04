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

using namespace std;
using namespace cv;

// IMPORTANT SETTINGS
static const int SLEEP_S = 1;//Short Sleep(ms);
static const int SLEEP_M = 1.5;//Medium Sleep(ms);
static const int SLEEP_L = 2;//Long Sleep(ms);


static const int DICT_ID = 16;
static const float MARKER_LENGTH = 0.1;
static const string CAM_PARAM="camparm.yml";
//PROTOCOL DICTIONNARY


class computervision
{
public:
	int detectTag(int argc, char *argv[]);



private:

static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs);

};

#endif /*COMPUTERVISION*/

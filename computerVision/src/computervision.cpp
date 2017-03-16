/*
By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install,
copy or use the software.

                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)

Copyright (C) 2013, OpenCV Foundation, all rights reserved.
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are
disclaimed. In no event shall copyright holders or contributors be liable for
any direct, indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/


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
*	ComputerVision function that permits the detection of ARUCO tags and the camera pose from this tag.
*
* NOTES:
*	This function is strongly based upon the detect.cpp code provided in the aruco module of opencv 3.2.0
* //https://stackoverflow.com/questions/18637494/camera-position-in-world-coordinate-from-cvsolvepnp#18643735
*
*********************************************************************************/

#include "computervision.h"


computervision::computervision()
{
}

computervision::computervision(string camParam)
{
    mCamParam=camParam;
}


bool computervision::readCameraParameters(std::string filename, Mat &camMatrix, Mat &distCoeffs) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}


bool computervision::detectTag(int aMaxTagID, int &tagID, double &xCam, double &zCam, double &angle) {
    
    //CommandLineParser parser(argc, argv, keys);
    //parser.about(about);
    bool flagCV = false;

    //Markers in the camera's frame.
    double xMarker, zMarker;

    Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
    detectorParams->doCornerRefinement = true; // do corner refinement in markers

    Ptr<aruco::Dictionary> dictionary =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(DICT_ID));

    Mat camMatrix, distCoeffs;
    /*
    Read the cameraParameters in order to get the the matrixs
    for the camera and distCoeffs.
    */
    cout << mCamParam << endl;

    bool readOk = readCameraParameters(mCamParam, camMatrix, distCoeffs);
    if(!readOk) {
        cerr << "Invalid camera file" << endl;
        return 0;
        }

    cv::VideoCapture inputVideo;

    cout << "Video opening" << endl;
    inputVideo.open(0);
    int waitTime = 10;

    clock_t flagT = clock();

    while(inputVideo.grab() && !flagCV && ((clock()-flagT)/(double) CLOCKS_PER_SEC)<TIME_DELAY) {
        cv::Mat image, imageCopy;
        inputVideo.retrieve(image);


        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;
        vector< Vec3d > rvecs, tvecs;
        // detect markers and estimate pose
        cv::aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
        image.copyTo(imageCopy);
	  if(ids.size() > 0 && ids[0]>0 && ids[0]<= aMaxTagID){
	  
          cv::aruco::estimatePoseSingleMarkers(corners, MARKER_LENGTH, camMatrix, distCoeffs, rvecs,tvecs);

          //Set the tagID reference to what was detected.
          tagID=ids[0];

          xMarker = tvecs[0][0];
          zMarker = tvecs[0][2];
          angle = atan2(zMarker,xMarker)*180/3.141596;

	        cv::Mat R;
          //Rodrigues Transformation
	        cv::Rodrigues(rvecs, R); // R is 3x3

	        cv::Mat Qtr(4, 4, R.type());
	        for(int i=0;i<3;i++)
	        {
	          for(int j=0;j<3;j++)
		        {

		          Qtr.at<double>(i,j)=R.at<double>(i,j);
		        }
	        }


	        Qtr.at<double>(0,3)=tvecs[0][0];
	        Qtr.at<double>(1,3)=tvecs[0][1];
	        Qtr.at<double>(2,3)=tvecs[0][2];
          Qtr.at<double>(3,0)=0;
          Qtr.at<double>(3,1)=0;
          Qtr.at<double>(3,2)=0;
          Qtr.at<double>(3,3)=1;

    	    Qtr=Qtr.inv();
          xCam = Qtr.at<double>(0,3)*100;
          zCam = Qtr.at<double>(2,3)*100;

          flagCV = true;

	    //cout << " 11: " << Qtr.at<double>(0,0) << " 12: " << Qtr.at<double>(0,1) << " 13: " << Qtr.at<double>(0,2) <<"1,4 "  << Qtr.at<double>(0,3) << endl;
	    //cout << " 21: " << Qtr.at<double>(1,0) << " 22: " << Qtr.at<double>(1,1) << " 23: " << Qtr.at<double>(1,2) <<"2,3 "  << Qtr.at<double>(1,3) << endl;
	    //cout << " 31: " << Qtr.at<double>(2,0) << " 32: " << Qtr.at<double>(2,1) << " 33: " << Qtr.at<double>(2,2) <<"3,4 "  << Qtr.at<double>(2,3) << endl;
        }

        imshow("out", imageCopy);

	waitKey(waitTime);
    }
    destroyWindow("out");
    return flagCV;
}

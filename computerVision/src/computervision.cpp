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
*
*
*********************************************************************************/


#include "computervision.h"


static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}


int detectTag(int argc, char *argv[]) {
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    detectorParams->doCornerRefinement = true; // do corner refinement in markers

    Ptr<aruco::Dictionary> dictionary =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(DICT_ID));

    Mat camMatrix, distCoeffs;
    if(estimatePose) {
        bool readOk = readCameraParameters(CAM_PARAM, camMatrix, distCoeffs);
        if(!readOk) {
            cerr << "Invalid camera file" << endl;
            return 0;
        }
    }

    VideoCapture inputVideo;
    int waitTime;
    if(!video.empty()) {
        inputVideo.open(video);
        waitTime = 0;
    } else {
        inputVideo.open(camId);
        waitTime = 10;
    }

    double totalTime = 0;
    int totalIterations = 0;

    while(inputVideo.grab()) {
        Mat image, imageCopy;
        inputVideo.retrieve(image);

        double tick = (double)getTickCount();

        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;
        vector< Vec3d > rvecs, tvecs;
        // detect markers and estimate pose
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
        if(estimatePose && ids.size() > 0)
            aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs,
                                             tvecs);
        double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
        totalTime += currentTime;
        totalIterations++;
        if(totalIterations % 30 == 0) {
            cout << "Detection Time = " << currentTime * 1000 << " ms "
                 << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
        }

        // draw results
        image.copyTo(imageCopy);
        if(ids.size() > 0) {
            aruco::drawDetectedMarkers(imageCopy, corners, ids);

            if(estimatePose) {
                for(unsigned int i = 0; i < ids.size(); i++)
                    aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i],
                                    markerLength * 0.5f);
            }
        }

        if(showRejected && rejected.size() > 0)
            aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));

        imshow("out", imageCopy);
        char key = (char)waitKey(waitTime);
        if(key == 27) break;

	if(ids.size() > 0)
	{
	    cout << "rotation x axis= " << rvecs[0][0] << endl;
	    cout << "translation x axis= " << tvecs[0][0] << endl;
	    cout << "rotation y axis= " << rvecs[0][1] << endl;
            cout << "translation y axis= " << tvecs[0][1] << endl;
	    cout << "rotation z axis= " << rvecs[0][2] << endl;
            cout << "translation z axis= " << tvecs[0][2] << endl;

	    cv::Mat R;
	    cv::Rodrigues(rvecs, R); // R is 3x3
	    R=tvecs*R;
	    cout << " 11: " << R.at<double>(0,0) << " 12: " << R.at<double>(0,1) << " 13: " << R.at<double>(0,2) << endl;
	    cout << " 21: " << R.at<double>(1,0) << " 22: " << R.at<double>(1,1) << " 23: " << R.at<double>(1,2) << endl;
	    cout << " 31: " << R.at<double>(2,0) << " 32: " << R.at<double>(2,1) << " 33: " << R.at<double>(2,2) << endl;
//	    R = R.t();  // rotation of inverse
//	    tvecs = -R * tvecs; // translation of inverse

//	    cv::Mat T(4, 4, R.type()); // T is 4x4
//	    T( cv::Range(0,3), cv::Range(0,3) ) = R * 1; // copies R into T
//	    T( cv::Range(0,3), cv::Range(3,4) ) = tvecs * 1; // copies tvec into T
	    // fill the last row of T (NOTE: depending on your types, use float or double)
//	    double *p = T.ptr<double>(3);
//	    p[0] = p[1] = p[2] = 0; p[3] = 1;
	}
    }

    return 0;
}

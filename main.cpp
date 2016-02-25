/*
 * main.cpp
 *
 *  Created on: Jan 22, 2016
 *      Author: hrvoje
 */

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs.hpp" //imread now part of this module (not corrected in opencv documentation)
#include "opencv2/video/video.hpp"

#include "opencv2/core.hpp"


#include <iostream>
#include <cmath>
#include <stdio.h>
#include <stdexcept>

#include "lucas_kanade.h"
#include "showFlow.h"

#include "calcErrorMetrics.h"
#include "time.h"
#include "optFlow_opencv.h"
#include "optFlow_paparazzi.h"
#include "read_dir_contents.h"


using namespace cv;
using namespace std;


int main()
{
	/*//Enter the path for images
	char firstImg[] =
			"/home/hrvoje/Desktop/Lucas Kanade algorithm/developing_LK/test_images/testSet5/frame10.png";
	char secondImg[] =
			"/home/hrvoje/Desktop/Lucas Kanade algorithm/developing_LK/test_images/testSet5/frame11.png";
	char ground_truth_file[] =
			"/home/hrvoje/Desktop/Lucas Kanade algorithm/developing_LK/test_images/testSet5/ground_truth.flo";
*/
	vector<string> *image_filenames;
	vector<string> *ground_truth_filenames;

	image_filenames = listdir("/home/hrvoje/Desktop/Lucas Kanade algorithm/developing_LK/test_images/testSequence2/images");
	ground_truth_filenames = listdir("/home/hrvoje/Desktop/Lucas Kanade algorithm/developing_LK/test_images/testSequence2/ground_truth");
	//for (vector<string>::const_iterator it = ground_truth_filenames->begin(); it!=ground_truth_filenames->end(); it++)
	//	cout << *it << endl;

	vector<string>::const_iterator image_files, ground_truth_files;
	image_files = image_filenames->begin() + 2;
	ground_truth_files = ground_truth_filenames->begin() + 2;

	string::size_type len = (*image_files).size() > (*ground_truth_files).size() ? (*image_files).size() : (*ground_truth_files).size();
	char firstImg[len];
	char secondImg[len];
	char ground_truth_file[len];
	strcpy(firstImg, (*image_files).c_str());
	image_files++;
	int frame = 1;

	while (image_files != image_filenames->end())
	{

	strcpy(secondImg, (*image_files).c_str());
	strcpy(ground_truth_file, (*ground_truth_files).c_str());

	cout << "Frames " << frame << " - " << frame+1<< endl;
	//Find good features to track
	//Initalize some constants and parameters
	const int MAX_FEATURES = 150;
	//int pyrLevel = 2;	 // max value is 3; 0 == pyramids not used(1 lvl), 1 == 2 levels used
	Mat curFrame = imread(firstImg, IMREAD_GRAYSCALE);
	vector<Point2f> currPoints; //typedef Point_<float> Point2f;
	//REMEMBER! currPoints.x == width == columns; currPoints.y == height == rows;

	//Find good features location
	goodFeaturesToTrack(curFrame, currPoints, MAX_FEATURES, 0.01, 10, Mat(), 3,	0, 0.04);

	//Refine good features location - finds subpixels feature location
	//cornerSubPix(currFrame, currPoints, subPixWinSize, Size(-1,-1), termcrit);
/*
	cout << "Points for both algorithms (column -- row)" << endl;
	cout << "size : " << currPoints.size() << endl;
	for (unsigned int i = 0; i != currPoints.size(); i++)
		cout << currPoints[i].x << "   " << currPoints[i].y << endl;
*/

	flowResults dataOpencv, dataPaparazzi, dataOpencvPyr;



	optFlow_paparazzi(firstImg, secondImg, ground_truth_file, currPoints, dataPaparazzi);
	//optFlow_opencv(firstImg, secondImg, ground_truth_file, currPoints, 2, dataOpencv);

	//optFlow_opencv(firstImg, secondImg, ground_truth_file, currPoints, pyrLevel, dataOpencvPyr);
	cout << endl;


	cout << "Starting number of points: " << currPoints.size() << endl;
	cout << endl;
	cout << "Paparazzi results: " << endl;
	cout << "Number of points left: " << dataPaparazzi.points_left << endl;
	cout << "Average magnitude error: " << dataPaparazzi.magErr <<endl;
	cout << "Average angular error: " << dataPaparazzi.angErr << endl;
	cout << "Time passed in miliseconds: " << dataPaparazzi.time << endl;
/*
	cout << endl; cout << endl;
	cout << "OpenCV results: " << endl;
	cout << "Number of points left: " << dataOpencv.points_left << endl;
	cout << "Average magnitude error: " << dataOpencv.magErr << endl;
	cout << "Average angular error: " << dataOpencv.angErr << endl;
	cout << "Time passed in miliseconds: " << dataOpencv.time << endl;
	cout << "=====================================================" << endl;
*/
	/*cout << endl; cout << endl;

	cout << "OpenCV with pyramids results: " << endl;
	cout << "Average magnitude error: " << dataOpencvPyr.magErr << endl;
	cout << "Average angular error: " << dataOpencvPyr.angErr << endl;
	cout << "Time passed in miliseconds: " << dataOpencvPyr.time << endl;*/

/*
	namedWindow("Paparazzi optical flow", WINDOW_AUTOSIZE);
	namedWindow("OpenCV optical flow", WINDOW_AUTOSIZE);
	//namedWindow("OpenCV with pyramids optical flow", WINDOW_AUTOSIZE);
	imshow("Paparazzi optical flow", dataPaparazzi.flow_viz);
	imshow("OpenCV optical flow", dataOpencv.flow_viz);
	//imshow("OpenCV with pyramids optical flow", dataOpencvPyr.flow_viz);
	waitKey();
*/
	strcpy(firstImg, (*image_files).c_str());
	image_files++;
	ground_truth_files++;
	frame++;
	}


	return 0;
}

/*
 * void goodFeaturesToTrack(InputArray image, OutputArray corners, int maxCorners, double qualityLevel,
 * 							double minDistance, InputArray mask=noArray(), int blockSize=3, bool useHarrisDetector=false,
 * 							double k=0.04 )
 *
 * 	image – Input 8-bit or floating-point 32-bit, single-channel image.
 * 	eig_image – The parameter is ignored.
 * 	temp_image – The parameter is ignored.
 * 	corners – Output vector of detected corners.
 * 	maxCorners – Maximum number of corners to return. If there are more corners than are found, the strongest of them is returned.
 * 	qualityLevel – Parameter characterizing the minimal accepted quality of image corners. The parameter value is multiplied by the best corner quality measure,
 * 					which is the minimal eigenvalue (see cornerMinEigenVal() ) or the Harris function response (see cornerHarris() ).
 * 					The corners with the quality measure less than the product are rejected. For example, if the best corner has the quality measure = 1500,
 * 					and the qualityLevel=0.01 , then all the corners with the quality measure less than 15 are rejected.
 * 	minDistance – Minimum possible Euclidean distance between the returned corners.
 * 	mask – Optional region of interest. If the image is not empty (it needs to have the type CV_8UC1 and the same size as image ),
 * 			 it specifies the region in which the corners are detected.
 * 	blockSize – Size of an average block for computing a derivative covariation matrix over each pixel neighborhood. See cornerEigenValsAndVecs() .
 * 	useHarrisDetector – Parameter indicating whether to use a Harris detector (see cornerHarris()) or cornerMinEigenVal().
 * 	k – Free parameter of the Harris detector.
 */


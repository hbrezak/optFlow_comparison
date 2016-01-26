/*
 * optFlow_opencv.cpp
 *
 *  Created on: Jan 22, 2016
 *      Author: hrvoje
 */

#include "opencv2/imgcodecs.hpp" //imread now part of this module (not corrected in opencv documentation)
#include "opencv2/video/video.hpp"
#include "opencv2/core.hpp"

#include <iostream>
#include <stdexcept>


#include "showFlow.h"
#include <time.h>
#include "optFlow_opencv.h"

using namespace cv;
using namespace std;


void optFlow_opencv(char* curImagePath, char* nextImagePath, char* groundTruthPath, const vector<Point2f>& currPoints, int pyrLevel, flowResults& results )
{
	TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
	Size subPixWinSize(10, 10), winSize(31, 31);
	vector<Point2f> nextPoints; //typedef Point_<float> Point2f;
	//REMEMBER! currPoints.x == width == columns; currPoints.y == height == rows;
	vector<uchar> status;
	vector<float> err;



	//Read images into openCV Mat image container, automatically convert to gray
	Mat currFrame, nextFrame;
	currFrame = imread(curImagePath,	IMREAD_GRAYSCALE);
	nextFrame = imread(nextImagePath,	IMREAD_GRAYSCALE);


	if (!currFrame.data || !nextFrame.data)
		throw invalid_argument ("Images have not loaded properly!");


	double time = (double)getTickCount();
	//Based on selected features find their position in next frame
	calcOpticalFlowPyrLK(currFrame, nextFrame, currPoints, nextPoints, status, err, winSize, pyrLevel, termcrit, 0, 0.001);
	time = (((double)getTickCount() - time)/getTickFrequency())*1000;
	results.time = time;





	vector<flow_t_> lk_flow;
	flow_t_ var;
	//DEBUG Output current feature locations and next feature locations
	//cout << "Format: row -- column" << endl;
	for (vector<Point2f>::size_type i = 0; i!= currPoints.size(); i++){
		//cout << currPoints[i].y << "--" << currPoints[i].x << "        " << nextPoints[i].y << "--" << nextPoints[i].x << endl;
		var.pos.x = currPoints[i].x;
		var.pos.y = currPoints[i].y;
		var.flow_x = nextPoints[i].x - currPoints[i].x;
		var.flow_y = nextPoints[i].y - currPoints[i].y;
		lk_flow.push_back(var);
	}

	calcErrorMetrics(groundTruthPath, lk_flow, results.angErr, results.magErr);

	results.flow_viz = showFlow(currFrame, nextFrame, curImagePath, lk_flow);
}
/* C++: void calcOpticalFlowPyrLK(InputArray prevImg, InputArray nextImg, InputArray prevPts, InputOutputArray nextPts,
 * 									OutputArray status, OutputArray err, Size winSize=Size(21,21), int maxLevel=3,
 * 									TermCriteria criteria=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01),
 * 									int flags=0, double minEigThreshold=1e-4 )
 *
 * prevImg – first 8-bit input image or pyramid constructed by buildOpticalFlowPyramid().
 * nextImg – second input image or pyramid of the same size and the same type as prevImg.
 * prevPts – vector of 2D points for which the flow needs to be found; point coordinates must be single-precision floating-point numbers.
 * nextPts – output vector of 2D points (with single-precision floating-point coordinates) containing the calculated new positions of input features in the second image;
 * 			  when OPTFLOW_USE_INITIAL_FLOW flag is passed, the vector must have the same size as in the input.
 * status – output status vector (of unsigned chars); each element of the vector is set to 1
 * 			if the flow for the corresponding features has been found, otherwise, it is set to 0.
 * err – output vector of errors; each element of the vector is set to an error for the corresponding feature,
 * 			type of the error measure can be set in flags parameter; if the flow wasn’t found then the error is not defined
 * 			(use the status parameter to find such cases).
 * winSize – size of the search window at each pyramid level.
 * maxLevel – 0-based maximal pyramid level number; if set to 0, pyramids are not used (single level), if set to 1, two levels are used, and so on;
 * 			  if pyramids are passed to input then algorithm will use as many levels as pyramids have but no more than maxLevel.
 * criteria – parameter, specifying the termination criteria of the iterative search algorithm
 * 				(after the specified maximum number of iterations criteria.maxCount or when the search window moves by less than criteria.epsilon.
 * flags –
 * operation flags:
 * 		OPTFLOW_USE_INITIAL_FLOW uses initial estimations, stored in nextPts; if the flag is not set, then prevPts is copied to nextPts and is considered the initial estimate.
 * 		OPTFLOW_LK_GET_MIN_EIGENVALS use minimum eigen values as an error measure (see minEigThreshold description);
 * 						if the flag is not set, then L1 distance between patches around the original and a moved point, divided by number of pixels in a window, is used as a error measure.
 * minEigThreshold – the algorithm calculates the minimum eigen value of a 2x2 normal matrix of optical flow equations
 * 						(this matrix is called a spatial gradient matrix in [Bouguet00]), divided by number of pixels in a window;
 * 						if this value is less than minEigThreshold, then a corresponding feature is filtered out and its flow is not processed,
 * 						so it allows to remove bad points and get a performance boost.
 */


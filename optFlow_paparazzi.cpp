/*
 * optFlow_paparazzi.cpp
 *
 *  Created on: Jan 22, 2016
 *      Author: hrvoje
 */

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include <stdexcept>

#include <iostream>
#include "rgb2yuv422.h"
extern "C" {
#include "fast_rosten.h"
#include "lucas_kanade.h"
}

#include "time.h"
#include "showFlow.h"
#include "optFlow_paparazzi.h"

using namespace cv;
using namespace std;

void optFlow_paparazzi(const char* curImagePath, const char* nextImagePath, const char* groundTruthPath, const vector<Point2f>& points, flowResults& results)
{
	Mat curImg = imread(curImagePath, CV_LOAD_IMAGE_COLOR);
	Mat nextImg = imread(nextImagePath, CV_LOAD_IMAGE_COLOR);


	image_t curYUV;
	image_t nextYUV;
	image_create(&curYUV, uint16_t (curImg.cols), uint16_t (curImg.rows), IMAGE_YUV422);
	image_create(&nextYUV, uint16_t (nextImg.cols), uint16_t (nextImg.rows), IMAGE_YUV422);

	// Convert RGB image to YUV 4:2:2 format and place it in curYUV/nextYUV
	if ( rgb2yuv422(curImg, &curYUV) )
		throw runtime_error ("Image conversion failed! Exiting...");

	if ( rgb2yuv422(nextImg, &nextYUV) )
		throw runtime_error ("Image conversion failed! Exiting...");

	// Create grayscale image from yuv image
	image_t curGray;
	image_t nextGray;
	image_create(&curGray, curYUV.w, curYUV.h, IMAGE_GRAYSCALE);
	image_create(&nextGray, nextYUV.w, nextYUV.h, IMAGE_GRAYSCALE);
	image_to_grayscale(&curYUV, &curGray);
	image_to_grayscale(&nextYUV, &nextGray);


	struct point_t corners[points.size()];
	//REMEMBER! points.x == width == columns; points.y == height == rows;
	int index = -1;
	for (vector<Point2f>::const_iterator iter = points.begin(); iter!=points.end(); iter++){
		index++;
		corners[index].x = iter->x; // column
		corners[index].y = iter->y; // row
		// in paparazzi, y iterates over height, x over width -> y are rows, x are cols
		// rows and cols are 0-based
	}

	uint16_t numTracked = (sizeof(corners)/sizeof(*corners));
	uint16_t window_size = 10; // za ovu 31 vrijednost rezultati fantasticni
	uint32_t subpixel_factor = 1000; //changed 16 -> 32 here, lucas_kanade.c, lucas_kanade.h; also all functions that use subpixel_factor: image subpixel window,
	uint8_t max_iterations = 20;
	uint8_t step_threshold = 3;
	uint8_t pyramid_level = 2; // 0 for no pyramids
	uint16_t max_track_corners = sizeof(corners)/sizeof(*corners);
	vector<flow_t_> lk_flow;
	flow_t_ var;

	/* good settings:
		 * uint16_t window_size = 31; // za ovu 31 vrijednost rezultati fantasticni
		 * 	uint32_t subpixel_factor = 10000;
		 * 	uint8_t max_iterations = 40;
		 * 	uint8_t step_threshold = 0.03;
	*/


	double time = (double)getTickCount();
	struct flow_t *vectors = opticFlowLK(&nextGray, &curGray, corners, &numTracked,
	                                       window_size / 2, subpixel_factor, max_iterations,
										   step_threshold, max_track_corners, pyramid_level);
	time = (((double)getTickCount() - time)/getTickFrequency())*1000; //in miliseconds
	results.time = time;

	//cout << endl;
	//cout << "Paparazzi tracked points (column -- row)" << endl;
	//cout << "Total number of points: "<< numTracked << endl;

	// Go through all the points
	for (uint16_t i = 0; i < numTracked; i++) {
		//because opticalFlowLK leaves out some corners
		var.pos.x = vectors[i].pos.x / subpixel_factor;
		var.pos.y = vectors[i].pos.y / subpixel_factor;
		var.flow_x = float(vectors[i].flow_x) / subpixel_factor;
		var.flow_y = float(vectors[i].flow_y) / subpixel_factor;
		lk_flow.push_back(var);
		//cout << var.flow_x << " " << var.flow_y << endl;
	}

/*
	cout << endl;
	cout << "Paparazzi flow (hor_flow -- vert_flow)" << endl;
	for (vector<flow_t_>::const_iterator it = lk_flow.begin();
			it != lk_flow.end(); it++)
		cout << it->flow_x << " " << it->flow_y << endl;
	cout << endl;
*/

	calcErrorMetrics(groundTruthPath, lk_flow, results.angErr, results.magErr);

	results.points_left = numTracked;

	//Vizualise calculated optical flow with arrow field
	results.flow_viz = showFlow(curImg, nextImg, curImagePath, lk_flow);


	image_free(&nextGray);
	image_free(&curGray);
	image_free(&nextYUV);
	image_free(&curYUV);
}

/**
 * Compute the optical flow of several points using the Lucas-Kanade algorithm by Yves Bouguet
 * The initial fixed-point implementation is doen by G. de Croon and is adapted by
 * Freek van Tienen for the implementation in Paparazzi.
 * @param[in] *new_img The newest grayscale image (TODO: fix YUV422 support)
 * @param[in] *old_img The old grayscale image (TODO: fix YUV422 support)
 * @param[in] *points Points to start tracking from
 * @param[in,out] points_cnt The amount of points and it returns the amount of points tracked
 * @param[in] half_window_size Half the window size (in both x and y direction) to search inside
 * @param[in] subpixel_factor The subpixel factor which calculations should be based on
 * @param[in] max_iterations Maximum amount of iterations to find the new point
 * @param[in] step_threshold The threshold at which the iterations should stop
 * @param[in] max_points The maximum amount of points to track, we skip x points and then take a point.
 * @return The vectors from the original *points in subpixels
 */
//struct flow_t *opticFlowLK(struct image_t *new_img, struct image_t *old_img, struct point_t *points, uint16_t *points_cnt,
//                         uint16_t half_window_size, uint16_t subpixel_factor, uint8_t max_iterations, uint8_t step_threshold, uint16_t max_points)



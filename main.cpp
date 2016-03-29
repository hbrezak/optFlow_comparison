/*
 * main.cpp
 *
 *  Created on: Jan 22, 2016
 *      Author: hrvoje
 */

#include <iostream>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/video.hpp"
#include <sstream>
#include <iomanip>

#include "read_dir_contents.h"
#include "optFlow_opencv.h"
#include "optFlow_paparazzi.h"
#include <iostream>
#include <fstream>

#include "rgb2yuv422.h"
extern "C" {
#include "fast_rosten.h"
#include "image.h"
}


// algorithms for detecting trackable features in images
enum find_points{
	GOOD_FEATURES,	// use openCV algorith goodFeaturesToTrack
	FAST			// use FAST algorithm
};


using namespace cv;
using namespace std;

int main()
{
	vector<string> *image_filenames;
	vector<string> *ground_truth_filenames;

	string testset_dir = "/home/hrvoje/Desktop/Lucas Kanade algorithm/developing_LK/test_images/testSequence3";
	image_filenames = listdir(testset_dir + "/images");
	ground_truth_filenames = listdir(testset_dir + "/ground_truth");
	string output_dir = testset_dir + "/output";


	//Initalize some constants and parameters
	find_points algorithm  = FAST;
	bool HAVE_GROUND_TRUTH = 1;
	bool SHOW_FLOW         = 0;
	bool SAVE_FLOW_IMAGES  = 0;
	bool PRINT_DEBUG_STUFF = 1;
	bool RESULTS_TO_FILE   = 0;
	const int MAX_POINTS   = 25;
	int thres = 20;
	vector<string>::const_iterator ground_truth_file = ground_truth_filenames->begin() + 2;
	int frame = 1;

	ofstream pointCount, avgMagErr, avgAngErr, time;

	if (RESULTS_TO_FILE) {
		string pointCount_dir = testset_dir + "/results/pointCount.txt";
		string avgMagErr_dir = testset_dir + "/results/avgMagErr.txt";
		string avgAngErr_dir = testset_dir + "/results/avgAngErr.txt";
		string time_dir = testset_dir + "/results/time.txt";

		pointCount.open(
				pointCount_dir.c_str(),
				std::ofstream::out | std::ofstream::trunc);
		avgMagErr.open(
				avgMagErr_dir.c_str(),
				std::ofstream::out | std::ofstream::trunc);
		avgAngErr.open(
				avgAngErr_dir.c_str(),
				std::ofstream::out | std::ofstream::trunc);
		time.open(
				time_dir.c_str(),
				std::ofstream::out | std::ofstream::trunc);
	}

	// Iterate through image files and calculate optical flow
	for (vector<string>::const_iterator image_file = image_filenames->begin() + 2; image_file != (image_filenames->end() - 1); image_file++)
	{
		//if (PRINT_DEBUG_STUFF)
			cout << "Frames " << frame << " - " << frame + 1 << endl;
			//if (frame == 2)break;

		const char *first_image = (*image_file).c_str();
		const char *second_image = (*(image_file + 1)).c_str();
		const char *ground_truth = (*ground_truth_file).c_str();
		ground_truth_file++;
		frame++;
		stringstream save_path;
		string type = ".jpg";

		Mat current_frame;
		vector<Point2f> points;


		switch (algorithm) {
		case GOOD_FEATURES:
		{
			//Find good points to track
			current_frame = imread(first_image, IMREAD_GRAYSCALE);
			goodFeaturesToTrack(current_frame, points, MAX_POINTS, 0.01, 10, Mat(), 3, 0, 0.04);
			break;
		}

		case FAST:
		{
			current_frame = imread(first_image, CV_LOAD_IMAGE_COLOR);

			image_t current_YUV;
			image_create(&current_YUV, uint16_t(current_frame.cols), uint16_t(current_frame.rows), IMAGE_YUV422);

			// Convert RGB image to YUV 4:2:2 format and place it in curYUV/nextYUV
			if (rgb2yuv422(current_frame, &current_YUV)) {
				printf("Image conversion failed! Exiting...");
				break;
			}

			// Create grayscale image from yuv image
			image_t current_gray_YUV;
			image_create(&current_gray_YUV, current_YUV.w, current_YUV.h, IMAGE_GRAYSCALE);
			image_to_grayscale(&current_YUV, &current_gray_YUV);
			uint16_t corner_cnt;

			// FAST corner detection (TODO: non fixed threshold)
			struct point_t *corners = fast9_detect(&current_gray_YUV, thres, 20, 0, 0, &corner_cnt);
			//printf("FAST points num: %u threshold: %d \n", corner_cnt, thres);

			 // Adaptive threshold
			if (1) {

				// Decrease and increase the threshold based on previous values
				if (corner_cnt < 40 && thres > 5) {
					thres--;
				} else if (corner_cnt > 50 && thres < 60) {
					thres++;
				}

			}

			float skip_points =	(corner_cnt > MAX_POINTS) ? (float)corner_cnt / MAX_POINTS : 1;
			uint16_t p;

			for (uint16_t i = 0; i < MAX_POINTS && i < corner_cnt; i++) {
				Point2f temp;
				p = i * skip_points;
				temp.x = corners[p].x; // column
				temp.y = corners[p].y; // row
				points.push_back(temp);
			}

			image_free(&current_YUV);
			image_free(&current_gray_YUV);
			break;
		}

		default:
			cout << "Error - please select algorithm for finding features."	<< endl;
			break;
		}


		/*cout << "Points for both algorithms (column -- row)" << endl;
		cout << "size : " << points.size() << endl;
		for (unsigned int i = 0; i != points.size(); i++)
			cout << i << "    " << points[i].x << "   " << points[i].y << endl;*/




		// Initalize containers for optical flow results
		flowResults dataOpencv, dataPaparazzi, dataOpencvPyr;

		// Calculate flow
		optFlow_paparazzi(first_image, second_image, ground_truth, points, dataPaparazzi, MAX_POINTS, HAVE_GROUND_TRUTH);
		optFlow_opencv(first_image, second_image, ground_truth, points, 2, dataOpencv, HAVE_GROUND_TRUTH);

		// Output flow to console
		if (PRINT_DEBUG_STUFF) {
			cout << endl;
			cout << "Starting number of points: " << points.size() << endl;
			cout << endl;
			cout << "Paparazzi results: " << endl;
			cout << "Number of points left: " << dataPaparazzi.points_left<< endl;
			if (HAVE_GROUND_TRUTH) {
				cout << "Average magnitude error: " << dataPaparazzi.magErr	<< endl;
				cout << "Average angular error: " << dataPaparazzi.angErr << endl;
			}
			cout << "Time passed in miliseconds: " << dataPaparazzi.time<< endl;

			cout << endl;
			cout << "OpenCV results: " << endl;
			cout << "Number of points left: " << dataOpencv.points_left << endl;
			if (HAVE_GROUND_TRUTH) {
				cout << "Average magnitude error: " << dataOpencv.magErr << endl;
				cout << "Average angular error: " << dataOpencv.angErr << endl;
			}
			cout << "Time passed in miliseconds: " << dataOpencv.time << endl;
			cout << "====================================================="
					<< endl;
		}
		if (SHOW_FLOW) {
			// Illustrate optical flow
			namedWindow("Paparazzi optical flow", WINDOW_AUTOSIZE);
			namedWindow("OpenCV optical flow", WINDOW_AUTOSIZE);
			imshow("Paparazzi optical flow", dataPaparazzi.flow_viz);
			imshow("OpenCV optical flow", dataOpencv.flow_viz);
			waitKey();
		}

		if (SAVE_FLOW_IMAGES){
			save_path << output_dir << "/paparazzi/flow_1" << setw(5) << setfill('0') << frame -1 << type;
			string filename = save_path.str();
			imwrite(filename, dataPaparazzi.flow_viz);
			save_path.str("");

			save_path << output_dir << "/opencv/flow_1" << setw(5) <<setfill('0') << frame - 1 << type;
			filename = save_path.str();
			imwrite(filename, dataOpencv.flow_viz);
			save_path.str("");
		}

		if (RESULTS_TO_FILE) {
			if (pointCount.is_open())
				pointCount << dataPaparazzi.points_left << " " << dataOpencv.points_left << endl;
			else
				cout << "Unable to open file";

			if (avgMagErr.is_open())
				avgMagErr << dataPaparazzi.magErr << " " << dataOpencv.magErr << endl;
			else
				cout << "Unable to open file";

			if (avgAngErr.is_open())
				avgAngErr << dataPaparazzi.angErr << " " << dataOpencv.angErr << endl;
			else
				cout << "Unable to open file";

			if (time.is_open())
				time << dataPaparazzi.time<< " " << dataOpencv.time << endl;
			else
				cout << "Unable to open file";
		}
	}

	if (RESULTS_TO_FILE) {
		if (pointCount.is_open())
			pointCount.close();

		if (avgMagErr.is_open())
			avgMagErr.close();

		if (avgAngErr.is_open())
			avgAngErr.close();

		if (time.is_open())
			time.close();
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


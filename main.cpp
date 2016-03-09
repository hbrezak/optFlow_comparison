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

#include "read_dir_contents.h"
#include "optFlow_opencv.h"
#include "optFlow_paparazzi.h"

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

	image_filenames = listdir("/home/hrvoje/Desktop/Lucas Kanade algorithm/developing_LK/test_images/testSequence3/images");
	ground_truth_filenames = listdir("/home/hrvoje/Desktop/Lucas Kanade algorithm/developing_LK/test_images/testSequence3/ground_truth");
	string output_dir = "/home/hrvoje/Desktop/Lucas Kanade algorithm/developing_LK/test_images/testSequence3/output";

	//Initalize some constants and parameters
	vector<string>::const_iterator ground_truth_file = ground_truth_filenames->begin() + 2;
	int frame = 1;
	const int MAX_POINTS = 250;


	// Iterate through image files and calculate optical flow
	for (vector<string>::const_iterator image_file = image_filenames->begin() + 2; image_file != (image_filenames->end() - 1); image_file++)
	{
		cout << "Frames " << frame << " - " << frame + 1 << endl;

		const char *first_image = (*image_file).c_str();
		const char *second_image = (*(image_file + 1)).c_str();
		const char *ground_truth = (*ground_truth_file).c_str();
		ground_truth_file++;
		frame++;
		stringstream save_path;
		string type = ".jpg";

		//Find good points to track
		Mat current_frame = imread(first_image, IMREAD_GRAYSCALE);
		vector<Point2f> points; //typedef Point_<float> Point2f;

		goodFeaturesToTrack(current_frame, points, MAX_POINTS, 0.01, 10, Mat(),	3, 0, 0.04);

		/*
		 cout << "Points for both algorithms (column -- row)" << endl;
		 cout << "size : " << currPoints.size() << endl;
		 for (unsigned int i = 0; i != currPoints.size(); i++)
		 cout << currPoints[i].x << "   " << currPoints[i].y << endl;
		 */

		// Initalize containers for optical flow results
		flowResults dataOpencv, dataPaparazzi, dataOpencvPyr;

		// Calculate flow
		optFlow_paparazzi(first_image, second_image, ground_truth, points, dataPaparazzi);
		optFlow_opencv(first_image, second_image, ground_truth, points, 2, dataOpencv);

		// Output flow to console
		cout << endl;
		cout << "Starting number of points: " << points.size() << endl;
		cout << endl;
		cout << "Paparazzi results: " << endl;
		cout << "Number of points left: " << dataPaparazzi.points_left << endl;
		cout << "Average magnitude error: " << dataPaparazzi.magErr << endl;
		cout << "Average angular error: " << dataPaparazzi.angErr << endl;
		cout << "Time passed in miliseconds: " << dataPaparazzi.time << endl;

		cout << endl;
		cout << "OpenCV results: " << endl;
		cout << "Number of points left: " << dataOpencv.points_left << endl;
		cout << "Average magnitude error: " << dataOpencv.magErr << endl;
		cout << "Average angular error: " << dataOpencv.angErr << endl;
		cout << "Time passed in miliseconds: " << dataOpencv.time << endl;
		cout << "=====================================================" << endl;

		// Illustrate optical flow
		namedWindow("Paparazzi optical flow", WINDOW_AUTOSIZE);
		namedWindow("OpenCV optical flow", WINDOW_AUTOSIZE);
		imshow("Paparazzi optical flow", dataPaparazzi.flow_viz);
		imshow("OpenCV optical flow", dataOpencv.flow_viz);
		waitKey();


		/*save_path << output_dir << "/paparazzi/flow_" << frame -1 << type;
		string filename = save_path.str();
		imwrite(filename, dataPaparazzi.flow_viz);
		save_path.str("");*/

//		save_path << output_dir << "/opencv/flow_" << frame - 1 << type;
//		filename = save_path.str();
//		imwrite(filename, dataOpencv.flow_viz);
//		save_path.str("");

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


/*
 * showFlow.cpp
 *
 *  Created on: Jan 12, 2016
 *      Author: hrvoje
 */

#include <cmath>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "showFlow.h"




using namespace std;
using namespace cv;


Mat showFlow(const Mat& currFrame, const Mat& nextFrame, const string& imgPath, const vector<flow_t_>& lk_flow)
{
	static const double pi = 3.14159265358979323846;

	Mat flowField = imread(imgPath, IMREAD_UNCHANGED);

	/* For fun (and debugging :)), let's draw the flow field. */
	for (vector<flow_t_>::size_type i = 0; i < lk_flow.size(); i++) {

		/* If Pyramidal Lucas Kanade didn't really find the feature, skip it. */
		if (lk_flow[i].flow_x > 1000 || lk_flow[i].flow_y > 1000)
			continue;

		int line_thickness;
		line_thickness = 1;
		/* CV_RGB(red, green, blue) is the red, green, and blue components
		 * of the color you want, each out of 255.
		 */
		CvScalar line_color;
		line_color = CV_RGB(255, 0, 0);

		/* Let's make the flow field look nice with arrows. */

		/* The arrows will be a bit too short for a nice visualization because of the high framerate
		 * (ie: there's not much motion between the frames).  So let's lengthen them by a factor of 3.
		 */
		CvPoint p, q;
		p.x = (int) lk_flow[i].pos.x;
		p.y = (int) lk_flow[i].pos.y;
		q.x = (int) lk_flow[i].pos.x + lk_flow[i].flow_x;
		q.y = (int) lk_flow[i].pos.y + lk_flow[i].flow_y;

		double angle;
		angle = atan2((double) p.y - q.y, (double) p.x - q.x);
		double hypotenuse;
		hypotenuse = std::sqrt(pow((p.y - q.y), 2) + pow((p.x - q.x), 2));

		/* Here we lengthen the arrow by a factor of three. */
		q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
		q.y = (int) (p.y - 3 * hypotenuse * sin(angle));

		/* Now we draw the main line of the arrow. */
		/* "frame1" is the frame to draw on.
		 * "p" is the point where the line begins.
		 * "q" is the point where the line stops.
		 * "CV_AA" means antialiased drawing.
		 * "0" means no fractional bits in the center cooridinate or radius.
		 */
		line(flowField, p, q, line_color, line_thickness, CV_AA, 0);
		/* Now draw the tips of the arrow.  I do some scaling so that the
		 * tips look proportional to the main line of the arrow.
		 */
		p.x = (int) (q.x + 9 * cos(angle + pi / 4));
		p.y = (int) (q.y + 9 * sin(angle + pi / 4));
		line(flowField, p, q, line_color, line_thickness, CV_AA, 0);
		p.x = (int) (q.x + 9 * cos(angle - pi / 4));
		p.y = (int) (q.y + 9 * sin(angle - pi / 4));
		line(flowField, p, q, line_color, line_thickness, CV_AA, 0);
	}

	/*namedWindow("Current frame", WINDOW_AUTOSIZE);
	namedWindow("Next frame", WINDOW_AUTOSIZE);
	namedWindow("Optical flow", WINDOW_AUTOSIZE);
	imshow("Current frame", currFrame);
	imshow("Next frame", nextFrame);
	imshow("Optical flow", flowField);
	waitKey(0);*/

	return flowField;
}

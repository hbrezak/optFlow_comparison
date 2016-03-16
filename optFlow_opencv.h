/*
 * optFlow_opencv.h
 *
 *  Created on: Jan 22, 2016
 *      Author: hrvoje
 */

#ifndef OPTFLOW_OPENCV_H_
#define OPTFLOW_OPENCV_H_

/*struct flowResults {
	float angErr;
	float magErr;
	float time;
	cv::Mat flow_viz;
};*/

#include "calcErrorMetrics.h"
#include <vector>
#include "opencv2/core.hpp"

void optFlow_opencv(const char*, const char*, const char*, const std::vector<cv::Point2f>&, int pyrLevel, flowResults&, bool);

#endif /* OPTFLOW_OPENCV_H_ */

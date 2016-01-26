/*
 * optFlow_paparazzi.h
 *
 *  Created on: Jan 22, 2016
 *      Author: hrvoje
 */

#ifndef OPTFLOW_PAPARAZZI_H_
#define OPTFLOW_PAPARAZZI_H_

/*struct flowResults {
	float angErr;
	float magErr;
	float time;
	cv::Mat flow_viz;
};*/
#include "calcErrorMetrics.h"
#include <vector>
#include "opencv2/core.hpp"

void optFlow_paparazzi(char*, char*, char*, const std::vector<cv::Point2f>&, flowResults&);


#endif /* OPTFLOW_PAPARAZZI_H_ */

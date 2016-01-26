/*
 * calcErrorMetrics.h
 *
 *  Created on: Jan 22, 2016
 *      Author: hrvoje
 */

#ifndef CALCERRORMETRICS_H_
#define CALCERRORMETRICS_H_

#include <vector>
#include "readGroundTruth.h"

struct flowResults {
	float angErr;
	float magErr;
	float time;
	cv::Mat flow_viz;
};


void calcErrorMetrics(const char*, const std::vector<flow_t_>&, float&, float&);

#endif /* CALCERRORMETRICS_H_ */

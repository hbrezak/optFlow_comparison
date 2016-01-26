#ifndef SHOWFLOW_H_
#define SHOWFLOW_H_

#include <vector>
#include "readGroundTruth.h"



cv::Mat showFlow(const cv::Mat&, const cv::Mat&, const std::string&, const std::vector<flow_t_>&);



#endif /* SHOWFLOW_H_ */

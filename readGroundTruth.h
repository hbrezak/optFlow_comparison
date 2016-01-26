/*
 * readGroundTruth.h
 *
 *  Created on: Jan 21, 2016
 *      Author: hrvoje
 */

#ifndef READGROUNDTRUTH_H_
#define READGROUNDTRUTH_H_

#include <vector>
#include "opencv2/core.hpp"

/* Structure for saving ground truth optical flow. It contains coordinates of points at which the flow
 * is assessed and ground truth flow at that point.
 */


struct point_t_ {
	uint16_t x;             // The x coordinate of the point; x is indexing the columns ( 0 -> width); horizontal direction
	uint16_t y;             // The y coordinate of the point; y is indexing the rows ( 0 -> height); vertical direction
	};


/* Vector structure for point differences */
struct flow_t_ {
  struct point_t_ pos;         // The original position the flow comes from; ( y, x ) == ( row, column )
  float flow_x;             // The x direction flow in subpixels; horizontal direction flow
  float flow_y;             // The y direction flow in subpixels; vertical direction flow
};




void readGroundTruth(const char*, const std::vector<flow_t_>&, std::vector<flow_t_>&);

#endif /* READGROUNDTRUTH_H_ */

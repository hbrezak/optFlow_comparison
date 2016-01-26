/*
 * rgb2yuv422.h
 *
 *  Created on: Jan 18, 2016
 *      Author: hrvoje
 */

#ifndef RGB2YUV422_H_
#define RGB2YUV422_H_

extern "C"{
#include "image.h"
}

int rgb2yuv422(const cv::Mat&, struct image_t*);

#endif /* RGB2YUV422_H_ */

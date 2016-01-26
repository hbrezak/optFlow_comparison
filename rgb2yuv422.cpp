/*
 * rgb2yuv422.cpp
 *
 *  Created on: Jan 15, 2016
 *      Author: hrvoje
 */
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include <iostream>


#include "rgb2yuv422.h"

using namespace cv;
using namespace std;

int rgb2yuv422(const Mat& img, struct image_t *yuv422) {



	if (!img.data) {
		cout << "Image reading unsuccessful! Exiting.." << endl;
		return -1;
	}
	CV_Assert(img.depth() == CV_8U);

	int nRows = img.rows;
	int nCols = img.cols;
	//int nChannels = img.channels();

	int nPixels = nRows * nCols;
	int buffsize;

	if (nPixels % 2 == 0)
		buffsize = 2 * nPixels;
	else
		buffsize = 2 * nPixels + 1;

	yuv422->buf_size = buffsize;

	uchar *t = (uchar*)yuv422->buf;

	float B1, G1, R1;
	uchar Y1, U1, V1;
	uchar* p = img.data;

	bool writeFull = true;

	//This loop converts RGB image to YUV 4:2:2
	for (unsigned int i = 0; i!= uint(nPixels); ++i) {
		B1 = float(*p++);
		G1 = float(*p++);
		R1 = float(*p);

		Y1 = uchar(round(0.257 * R1 + 0.504 * G1 + 0.098 * B1 + 16));
		U1 = uchar(round(-0.148 * R1 - 0.291 * G1 + 0.439 * B1 + 128));
		V1 = uchar(round(0.439 * R1 - 0.368 * G1 - 0.071 * B1 + 128));

		if (writeFull) {
			*t++ = U1;
			*t++ = Y1;
			*t = V1;
		} else
			*t = Y1;

		writeFull = !writeFull;
		if (i < uint(nPixels)-1){
		t++;
		p++;
		}
	}
/*
    uchar *buff_pointer = (uchar*)yuv422->buf;
	cout << "(function) Writing YUV 4:2:2 image data: " << endl;
	for (unsigned int i = 0; i!=uint(yuv422->buf_size); ++i){
		cout << int(*buff_pointer) << endl;
		buff_pointer++;
	}
*/
	return 0;
}

/*
 * readGroundTruth.cpp
 *
 *  Created on: Jan 21, 2016
 *      Author: hrvoje
 */

#include <iostream>
#include <stdexcept>
#include "readGroundTruth.h"


using namespace cv;
using namespace std;

#define TAG_FLOAT 202021.25  // check for this when READING the file


void readGroundTruth(const char* filename, const vector<flow_t_>& points, vector<flow_t_>& gtFlow) {

	gtFlow.clear(); // Make sure we write in empty vector

	if (filename == NULL)
		throw domain_error("readGroundTruth : empty filename");

	const char *dot = strrchr(filename, '.');
	if (strcmp(dot, ".flo") != 0)
		throw invalid_argument("readGroundTruth : extension .flo expected");

	FILE *stream = fopen(filename, "rb");
	if (stream == 0)
		throw invalid_argument("readGroundTruth : could not open file");

	int width = 0;
	int height = 0;
	float tag = 0;

	if ((int) fread(&tag, sizeof(float), 1, stream) != 1
			|| (int) fread(&width, sizeof(int), 1, stream) != 1
			|| (int) fread(&height, sizeof(int), 1, stream) != 1)
		throw domain_error("readGroundTruth: problem reading file ");

	if (tag != TAG_FLOAT) // simple test for correct endian-ness
		throw domain_error(
				"readGroundTruth: wrong tag (possibly due to big-endian machine?)");

	// another sanity check to see that integers were read correctly (99999 should do the trick...)
	if (width < 1 || width > 99999)
		throw domain_error("readGroundTruth: illegal width ");

	if (height < 1 || height > 99999)
		throw domain_error("readGroundTruth: illegal height ");

	// number of floats in total is width*height*2 ( for each pixel we have u,v pair) so i know how many times i want to read but this value
	// might be bigger than uint for bigger images;

	vector<float> full_gt_flow_x, full_gt_flow_y;
	float hor_flow = 0;
	float vert_flow = 0;

	unsigned int n = width * height;

	// Flow order - horizontal (u) and vertical (v) flow components;
	// u[row0,col0], v[row0,col0], u[row0,col1], v[row0,col1], ...

	for (unsigned int i = 0; i < n; ++i) {
		if ((int) fread(&hor_flow, sizeof(float), 1, stream) != 1
				|| (int) fread(&vert_flow, sizeof(float), 1, stream) != 1)
			throw domain_error(
					"readGroundTruth: problem reading flow values from file");

		full_gt_flow_x.push_back(hor_flow);
		full_gt_flow_y.push_back(vert_flow);
	}

	//cout << "File read successfully!" << endl;

	fclose(stream);


	flow_t_ val;
	unsigned int index = 0;
	for (vector<flow_t_>::const_iterator iter = points.begin();
			iter != points.end(); ++iter) {
		index = (*iter).pos.y * width + (*iter).pos.x; // corrected indexing into one line stream
		val.pos.x = (*iter).pos.x; //col
		val.pos.y = (*iter).pos.y; //row
		val.flow_x = full_gt_flow_x.at(index); //horizontal flow
		val.flow_y = full_gt_flow_y.at(index); //vertical flow
		gtFlow.push_back(val);
	}

}


// read and write our simple .flo flow file format

// ".flo" file format used for optical flow evaluation
//
// Stores 2-band float image for horizontal (u) and vertical (v) flow components.
// Floats are stored in little-endian order.
// A flow value is considered "unknown" if either |u| or |v| is greater than 1e9.
//
//  bytes  contents
//
//  0-3     tag: "PIEH" in ASCII, which in little endian happens to be the float 202021.25
//          (just a sanity check that floats are represented correctly)
//  4-7     width as an integer
//  8-11    height as an integer
//  12-end  data (width*height*2*4 bytes total)
//          the float values for u and v, interleaved, in row order, i.e.,
//          u[row0,col0], v[row0,col0], u[row0,col1], v[row0,col1], ...
//

//velicina izlaznih arraya jest broj pixela slike

//strrchr Returns a pointer to the last occurrence of character in the C string str; not found -> returns null pointer

//int strcmp ( const char * str1, const char * str2 );
//Compares the C string str1 to the C string str2.
//This function starts comparing the first character of each string.
//If they are equal to each other, it continues with the following pairs until the characters differ or until a terminating null-character is reached.
//<0	the first character that does not match has a lower value in ptr1 than in ptr2
//0	the contents of both strings are equal
//>0	the first character that does not match has a greater value in ptr1 than in ptr2

//fopen is called with various modes, 'r' is mode for reading txt file; to read binary file mode is 'rb'

// sizeof returns size of expression in bytes; sizeof(float) = 4; sizeof(int) = 4;

/*
 * size_t fread ( void * ptr, size_t size, size_t count, FILE * stream );
 *
 * Read block of data from stream
 * Reads an array of count elements, each one with a size of size bytes, from the stream and stores them in the block of memory specified by ptr.
 * The position indicator of the stream is advanced by the total amount of bytes read.
 * The total amount of bytes read if successful is (size*count).
 *
 * The total number of elements successfully read is returned.
 * If this number differs from the count parameter, either a reading error occurred or the end-of-file was reached while reading.
 * In both cases, the proper indicator is set, which can be checked with ferror and feof, respectively.
 * If either size or count is zero, the function returns zero and both the stream state and the content pointed by ptr remain unchanged.
 * size_t is an unsigned integral type.
 */

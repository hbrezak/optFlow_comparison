/*
 * read_dir_contents.cpp
 *
 *  Created on: Feb 23, 2016
 *      Author: hrvoje
 */

/*
 * main.cpp
 *
 *  Created on: Feb 22, 2016
 *      Author: hrvoje
 */


#include <iterator>
#include <iostream>
#include <algorithm>
#include <sys/types.h>
#include <dirent.h>
#include "read_dir_contents.h"

using namespace std;

vector<string> *listdir(const char *dirname) {
  DIR *dp;
  dirent *d;
  vector<string> *vec = new vector<string>;

  dp = opendir(dirname);
  while((d = readdir(dp)) != NULL)
    vec->push_back(d->d_name);

  sort(vec->begin(), vec->end());
  return vec;
}





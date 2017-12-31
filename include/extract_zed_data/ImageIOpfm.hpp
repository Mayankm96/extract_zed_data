/*  Name:
 *      ImageIOpfm.h
 *
 *  Description:
 *      Used to read/write pfm images to and from
 *      opencv Mat image objects
 *
 *      Works with PF color pfm files and Pf grayscale
 *      pfm files
 *
 *  Source: https://github.com/dscharstein/pfmLib
 */

#ifndef __ImageIOpfm_H_INCLUDED__
#define __ImageIOpfm_H_INCLUDED__

#include "opencv2/opencv.hpp"
#include <iostream>

int readFilePFM(const std::string filename, cv::Mat & im);
int writeFilePFM(const std::string filename, const cv::Mat & im, float scalef);

#endif

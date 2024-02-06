/*********************************************
 * sobel.hpp
 * Matthew Wong
 * CPE 442 - Lab 3
 * 
 * Rev 1 - 1/30/24
 * Initial
 * Video Demo: https://youtu.be/28PkBCEMIcQ
*********************************************/
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;

#ifndef SOBEL_HEADER_H
#define SOBEL_HEADER_H

Mat to442_grayscale(Mat rgb_frame, int i=0, int n=1);

Mat to442_sobel(Mat frame, int i=0, int n=1);

#endif
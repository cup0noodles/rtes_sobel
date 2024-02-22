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
#include <arm_neon.h>
#include <opencv2/opencv.hpp>
#include <math.h>

using namespace cv;

#ifndef SOBEL_HEADER_H
#define SOBEL_HEADER_H

struct range
{
    int begin;
    int end;
};

Mat to442_grayscale(Mat rgb_frame, int i=0, int n=1);

Mat to442_sobel(Mat frame, int i=0, int n=1);

struct range* generateRange();

#endif
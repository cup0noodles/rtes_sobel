/*********************************************
 * imgview.h
 * Matthew Wong
 * CPE 442 - Lab 2
 * 
 * Rev 1 - 1/18/24
 * Initial
*********************************************/
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <math.h>

#ifndef _IMGVIEW_HEADER_HPP
#define _IMGVIEW_HEADER_HPP

using namespace cv;

//define globals
pthread_barrier_t allocatebarrier, displaybarrier; //common barrier used to gather all sobel threads

struct sobelArgs 
{
    int i; //thread number
    int n; //thread count
    Mat* allocated_frame;
    Mat* tout;
};

struct displayArgs
{
    int n; //thread count
    Mat* tout;
    Mat* allocated_frame;
    VideoCapture* cap;

};

void *sobelThread(void *sobelArgs);
void *displayThread(void *displayArgs);
void setupBarrier(int numThreads, pthread_barrier_t* localbarrier);



#endif
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
#include <time.h>

#ifndef _IMGVIEW_HEADER_HPP
#define _IMGVIEW_HEADER_HPP

using namespace cv;

//define globals
pthread_barrier_t allocatebarrier, displaybarrier; //common barrier used to gather all sobel threads
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

struct sobelArgs 
{
    int i; //thread number
    int n; //thread count
    Mat* allocated_frame;
    Mat* output_frame;
};

struct displayArgs
{
    int n; //thread count
    Mat* allocated_frame;
    Mat* output_frame;
    VideoCapture* cap;
    int debug_code;

};

void *sobelThread(void *sobelArgs);
void *displayThread(void *displayArgs);
void setupBarrier(int numThreads, pthread_barrier_t* localbarrier);



#endif
/*********************************************
 * imgview.c
 * Matthew Wong
 * CPE 442 - Lab 2
 * 
 * Rev 1 - 1/18/24
 *  Initial
 * 
 *  Video Demo: https://youtu.be/8W-FaKT6LJg
 * 
 * Rev 2 - 1/30/24
 *  Lab 3
 *  Added support for videos
 *  Implimented grayscale and sobel
 *  Video Demo: https://youtu.be/28PkBCEMIcQ
*********************************************/
#include <iostream>
#include <opencv2/opencv.hpp>

#include "sobel.hpp"

using namespace cv;

int main(int argc, char** argv)
{
    String file_path = argv[1];
    int img_wait = atoi(argv[2]);
    VideoCapture cap(file_path); 

    //check that video was actually opened
    if(!cap.isOpened())
    {
        std::cout << "Video not accessed" <<std::endl;
        exit(-1);
    }

    while(1)
    {
        Mat raw_frame, grayscale, s_out;
        cap >> raw_frame; //apply math to this frame
        if (raw_frame.empty()) break;
        //maths here
        grayscale = to442_grayscale(raw_frame);
        s_out = to442_sobel(grayscale);

        
        imshow(file_path, s_out);
        waitKey(img_wait); //allow network time to send image
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
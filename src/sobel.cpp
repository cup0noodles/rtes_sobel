/*********************************************
 * sobel.cpp
 * Matthew Wong
 * CPE 442 - Lab 3
 * 
 * Rev 1 - 1/30/24
 * Initial
 * Video Demo: https://youtu.be/28PkBCEMIcQ
*********************************************/
#include "sobel.hpp"

Mat to442_grayscale(Mat rgb_frame)
{
    int rows = rgb_frame.rows;
    int cols = rgb_frame.cols;

    Mat graymat(rows, cols, CV_8U, Scalar(0));
    for(int r=0;r<rows-1;r++)
    {
        for(int c=0;c<cols-1;c++)
        {
            unsigned char *p_rgb = rgb_frame.ptr(r, c); //BRG format
            unsigned char *p_gray = graymat.ptr(r, c);
            p_gray[0] = (uint8_t)((double)p_rgb[0]*0.0722 + (double)p_rgb[1]*0.2126 + (double)p_rgb[2]*0.7152);
        }
    }
    return graymat;
}

Mat to442_sobel(Mat frame)
{
    int rows = frame.rows;
    int cols = frame.cols;
    Mat sobel_frame(rows, cols, CV_8U, Scalar(0));
    // init 0 so no issues with edges

    //tighted bounds by 1 since filter reaches outwards
    //using simplified |G| = |Gx| + |Gy|
    //Equation is 
    // |G| = |(P1 + 2*P2 + P3) - (P7 + 2*P8 + P9)|
    //      +|(P3 + 2*P6 + P9) - (P1 + 2*P4 + P7)|
    for(int r=1;r<rows-2;r++)
    {
        for(int c=1;c<cols-2;c++)
        {
            unsigned char p[9];
            // build matrix of surrounding pixels
            for(int i=0;i<3;i++)
            {
                for(int j=0;j<3;j++)
                {
                    p[i*3+j] = *frame.ptr(r-(i-1),c-(j-1));
                }
            }
            unsigned char *s_p = sobel_frame.ptr(r,c);
            //peform convolution/addition
            s_p[0] = abs((p[0] + 2*p[1] + p[2]) - (p[6] + 2*p[7] + p[8])) \
                   + abs((p[2] + 2*p[5] + p[8]) - (p[0] + 2*p[3] + p[6]));

        }
    }
    return sobel_frame;
}
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

Mat to442_grayscale(Mat rgb_frame, int i, int n)
{
    int rows = rgb_frame.rows;
    int cols = rgb_frame.cols;
    // going to split bounds vertically (along cols)
    // generally video is wide format, going to use 16 threads tops, 4 on a pi
    // minimzes length of pixels that are going to be recomputed
    int range = (rows/n);
    int start = max((range*i)-1, 0);
    int end = min(range*(i+1)+2,rows);

    Mat graymat(rows, cols, CV_8U, Scalar(0));

    //constants
    uint8x8_t BLUE_F = vdup_n_u8(18);
    uint8x8_t RED_F = vdup_n_u8(54);
    uint8x8_t GREEN_F = vdup_n_u8(183);

    for(int r=start;r<end;r++)
    {
        
        for(int c=0;c<cols;c=c+8)
        {
            unsigned char *p_rgb = rgb_frame.ptr(r, c); //BRG format
            unsigned char *p_gray = graymat.ptr(r, c);

            uint8x8x3_t rgb;
            uint16x8_t acc = vdupq_n_u16(0);
            uint8x8_t grayscale;
            rgb = vld3_u8(p_rgb);
            // do all the multiplication, add to our two accumulation regs
            acc = vmlal_u8(acc, rgb.val[0], BLUE_F); //blue
            acc = vmlal_u8(acc, rgb.val[1], RED_F); //red
            acc = vmlal_u8(acc, rgb.val[2], GREEN_F); //green

            grayscale = vshrn_n_u16(acc, 8);
            vst1_u8(p_gray, grayscale);

            // p_gray[0] = (uint8_t)((double)p_rgb[0]*0.0722 + (double)p_rgb[1]*0.2126 + (double)p_rgb[2]*0.7152);
        }
    }
    return graymat;
}

Mat to442_sobel(Mat frame, int i, int n)
{
    int rows = frame.rows;
    int cols = frame.cols;

    int range = (rows/n);
    int start = max((range*i)-1, 0);
    int end = min(range*(i+1)+2,rows);

    Mat sobel_frame(rows, cols, CV_8U, Scalar(0));

    int16x8_t two_v = vdupq_n_s16(2);
    // init 0 so no issues with edges

    //tighted bounds by 1 since filter reaches outwards
    //using simplified |G| = |Gx| + |Gy|
   
    for(int r=start+1;r<end-1;r++)
    {
        for(int c=1;c<cols-1;c=c+8)
        {
            uint8x8_t s_out;
            int16x8_t s[9];

            int16x8_t acc0;
            int16x8_t acc1;
            int16x8_t outtemp;
            int16x8_t outtemp2;
            unsigned char *s_p = sobel_frame.ptr(r,c);
            // grab 8 across of pixels
            // 8, 3x3s, move 8 each time
            //calculate 6 at a time

            //get start to each row
            for (int i=0;i<3;i++)
            {
                for( int j=0; j<3;j++)
                {
                    uint8x8_t p = vld1_u8(frame.ptr(r+(i-1),c+(j-1)));
                    s[i*3+j] = vreinterpretq_s16_u16(vmovl_u8(p));
                }
            }

            //Equation is 
            // |G| = |(P1 + 2*P2 + P3) - (P7 + 2*P8 + P9)|
            //      +|(P3 + 2*P6 + P9) - (P1 + 2*P4 + P7)|
            
            acc0 = vdupq_n_s16(0);
            acc1 = vdupq_n_s16(0);
            outtemp = vdupq_n_s16(0);
            outtemp2 = vdupq_n_s16(0);

            //2*p1
            acc0 = vmulq_s16(two_v, s[1]);
            //+p0
            acc0 = vaddq_s16(acc0, s[0]);
            //+p[2]
            acc0 = vaddq_s16(acc0, s[2]);
            
            //2*p7
            acc1 = vmulq_s16(two_v, s[7]);
            //+p6
            acc1 = vaddq_s16(acc1, s[6]);
            //+p8
            acc1 = vaddq_s16(acc1, s[8]);
            
            outtemp = vabdq_s16(acc1, acc0);

            acc0 = vdupq_n_s16(0);
            acc1 = vdupq_n_s16(0);
            //2*p5
            acc0 = vmulq_n_s16(s[5],2);
            //+p2
            acc0 = vaddq_s16(acc0, s[2]);
            //+p8
            acc0 = vaddq_s16(acc0, s[8]);
            
            //2*0
            acc1 = vmulq_n_s16(s[3],2);
            //+p4
            acc1 = vaddq_s16(acc1, s[0]);
            //+p6
            acc1 = vaddq_s16(acc1, s[6]);
            
            outtemp2 = vsubq_s16(acc1, acc0);
           
            outtemp2 = vabsq_s16(outtemp2);
            outtemp = vabsq_s16(outtemp);

            outtemp = vaddq_s16(outtemp, outtemp2); //outtemp has 8 output pixels

            //peform convolution/addition

            uint16x8_t s_unsigned = vreinterpretq_u16_s16(outtemp);
            s_out = vmovn_u16(s_unsigned);
            vst1_u8(s_p, s_out);

        }
    }
    return sobel_frame;
}
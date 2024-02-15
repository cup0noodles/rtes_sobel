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
 * 
 * Rev 3 - 2/9/24
 *  Lab 4
 *  Added variable multithreading support with
 *  shared memory
 *  Note: Limiting factor here is X11. Will need
 *  to switch to better performance measure that
 *  is not dependant on X11.
 *  Video Demo: 
*********************************************/
#include <time.h>

#include "imgview.hpp"
#include "sobel.hpp"

int main(int argc, char** argv)
{
    String file_path = argv[1];
    int thread_count = atoi(argv[2]); //count of worker threads
    int debug_code = 0;
    if (argc == 4)
    {
        debug_code = atoi(argv[3]);
    }
    clock_t ts, te;
    if (debug_code == 1)
    {
        ts = clock();
    }
    
    VideoCapture cap(file_path); 
    //check that video was actually opened
    if(!cap.isOpened())
    {
        std::cout << "Video not accessed" <<std::endl;
        exit(-1);
    }
    // allocate image matrix
    Mat allocated_frame; //input matrix
    Mat output_frame; //thread output matrix

    pthread_t thread[thread_count+1];
    // int ret_val[thread_count+1];
    setupBarrier(thread_count+1, &allocatebarrier);
    setupBarrier(thread_count+1, &displaybarrier);

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);    

    //start all threads
    for (int i=0;i<thread_count+1;i++)
    {
        if(i==thread_count) //last thread is display
        {
            struct displayArgs da = 
            {
                .n=thread_count,
                .allocated_frame=&allocated_frame,
                .output_frame=&output_frame,
                .cap=&cap,
                .debug_code=debug_code
            };
            pthread_create(&thread[i], NULL, displayThread, (void *)&da);
        }
        else if(i < thread_count)
        {
            struct sobelArgs *sa = (struct sobelArgs*)malloc(sizeof(struct sobelArgs));
            sa->i=i;
            sa->n=thread_count;
            sa->allocated_frame=&allocated_frame;
            sa->output_frame=&output_frame;

            pthread_create(&thread[i], NULL, sobelThread, (void *)sa);
        }
    }

    //wait for all to finish and exit


    for (int i=0;i<thread_count+1;i++)
    {
        pthread_join(thread[i],NULL);
    }

    if(debug_code == 1)
    {
        te = clock();
        double tt = double(te - ts) / double(CLOCKS_PER_SEC);
        printf("Time Elapsed: %f\n",tt);
    }

    cap.release();
    destroyAllWindows();
    return 0;
}

void setupBarrier(int numThreads, pthread_barrier_t* localbarrier)
{
    pthread_barrier_init(localbarrier, NULL, numThreads);
}

void* sobelThread(void *sobelArgs)
{
    struct sobelArgs* sa = (struct sobelArgs*)sobelArgs;
    int tn = sa->i;
    //each frame uses allocated_frame, makes copy of grayscale and sobel
    //display thread will recombine the final output
    //will not be recombining at grayscale, just take the performance hit of pixel overlap
    //is more significant at higher thread counts potentially but fine here
    printf("TH%u: entered\n",tn);
    //generate mask
    int itr = 0;
    Mat emask, frame, gs, sob;
    while(1)
    {
        //wait for global to allocate
        //printf("TH%u: waiting for img allocate\n",tn);
        pthread_barrier_wait(&allocatebarrier);
        //grab frame from allocated_frame

        frame = *sa->allocated_frame;
        if (itr == 0)
        {
            int rows = frame.rows;
            int cols = frame.cols;
            int col_range = (cols/sa->n);
            emask = Mat(rows, cols, CV_8U, Scalar(0));
            int col_start = max((col_range*tn), 0);
            int col_end = min((col_range*(tn+1)),cols);
            for(int r=0;r<rows;r++)
            { 
                for(int c=col_start;c<col_end;c++)
                {
                    unsigned char *p_mask = emask.ptr(r,c);
                    p_mask[0] = 255;
                }
            }
        }
        //exit if empty frame
        if (frame.empty())
        {
            printf("TH%i: frame empty. exiting\n",tn);
            return 0;
        }
        //process grayscale
        gs = to442_grayscale(frame,sa->i,sa->n);
        //process sobel
        sob = to442_sobel(gs, sa->i, sa->n);

        pthread_barrier_wait(&displaybarrier);
        //wait for previous frame to be output before copying

        pthread_mutex_lock(&mutex);
        //could be benefitial to mask this later on for better mem performance
        sob.copyTo(*sa->output_frame,emask);
        pthread_mutex_unlock(&mutex);

        //loop
        itr += 1;
    }
}

void* displayThread(void *displayArgs)
{
    struct displayArgs* da = (struct displayArgs*)displayArgs;
    int itr = 0;
    int cols, rows;
    Mat masks[da->n];
    struct timespec start, end;
    uint64_t diff;

    while(1)
    {
    //allocate frame
    *da->cap >> *da->allocated_frame;
    // printf("DIS: Allocating...\n");

    //alocate empty output

    pthread_barrier_wait(&allocatebarrier); //wait for all frames to be ready to grab
    //exit if empty frame
    if((*da->allocated_frame).empty()) return 0;
    
    else if(itr == 0)
    {
        cols = (*da->allocated_frame).cols;
        rows = (*da->allocated_frame).rows;
        Mat newout (rows, cols, CV_8U, Scalar(0));
        newout.copyTo(*da->output_frame);
        (*da->output_frame) = Scalar(0);
        clock_gettime(CLOCK_MONOTONIC, &start);	/* mark start time */
        pthread_barrier_wait(&displaybarrier);
    }
    else if (itr != 0)
    {
        //skip if frame 0
        //wait for frames to be done with sobel

        if(da->debug_code == 0)
        {
            imshow("Sobel Output",*da->output_frame);
            waitKey(1); // needed for proper display
        }

        //clean output frame before continuing
        //(*da->output_frame) = Scalar(0);
       
        clock_gettime(CLOCK_MONOTONIC, &end);	/* mark start time */
        diff = 1000000000L * (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec);
        printf("frame time: %lu ms\r",diff/1000000);
        fflush(stdout);
        start = end;

        pthread_barrier_wait(&displaybarrier); 
        // once output frame is cleared threads can write again

    }
    itr += 1;
    //loop
    }
}
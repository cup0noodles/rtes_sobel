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
#include "imgview.hpp"
#include "sobel.hpp"

int main(int argc, char** argv)
{
    String file_path = argv[1];
    int thread_count = atoi(argv[2]); //count of worker threads
    
    VideoCapture cap(file_path); 
    //check that video was actually opened
    if(!cap.isOpened())
    {
        std::cout << "Video not accessed" <<std::endl;
        exit(-1);
    }
    // allocate image matrix
    Mat allocated_frame; //input matrix
    Mat tout[thread_count]; //thread output matrix

    pthread_t thread[thread_count+1];
    int ret_val[thread_count+1];
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
                .tout=tout,
                .allocated_frame=&allocated_frame,
                .cap=&cap
            };
            ret_val[i] = pthread_create(&thread[i], NULL, displayThread, (void *)&da);
        }
        else
        {
            struct sobelArgs sa = 
            {
                .i=i,
                .n=thread_count,
                .allocated_frame=&allocated_frame,
                .tout=&tout[i]
            };
            ret_val[i] = pthread_create(&thread[i], NULL, sobelThread, (void *)&sa);
        }
    }

    //wait for all to finish and exit


    for (int i=0;i<thread_count+1;i++)
    {
        pthread_join(thread[i],NULL);
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
    printf("TH%i: entered\n",tn);
    while(1)
    {
        //wait for global to allocate
        printf("TH%i: waiting for img allocate\n",tn);
        pthread_barrier_wait(&allocatebarrier);
        //grab frame from allocated_frame

        Mat frame = *sa->allocated_frame;
        //exit if empty frame
        if (frame.empty())
        {
            if(!tn) printf("TH%i: frame empty. exiting\n",tn);
            return 0;
        }
        //process grayscale
        Mat gs = to442_grayscale(frame,sa->i,sa->n);
        //process sobel
        Mat sob = to442_sobel(gs, sa->i, sa->n);
        printf("TH%i: waiting for display\n",tn);
        pthread_barrier_wait(&displaybarrier);
        //wait for previous frame to be output before copying
        *sa->tout = sob;
        //loop
    }
}

void* displayThread(void *displayArgs)
{
    struct displayArgs* da = (struct displayArgs*)displayArgs;
    int itr = 0;
    int col_range, col_start, col_end;
    Mat masks[da->n];
    while(1)
    {
    //allocate frame
    *da->cap >> *da->allocated_frame;
    printf("DIS: Allocating...\n");
    pthread_barrier_wait(&allocatebarrier); //wait for all frames to be ready to grab
    //exit if empty frame
    if((*da->allocated_frame).empty()) return 0;
    
    if(itr == 0)
    {
        printf("DIS: Initializing Masks...\n");
        int cols = (*da->allocated_frame).cols;
        int rows = (*da->allocated_frame).rows;
        //define bounds
        col_range = (cols/da->n);

        for(int i=0;i<da->n;i++)
        { //per thread
            Mat emask(rows, cols, CV_8U, Scalar(0));    
            col_start = max((col_range*i)-1, 0);
            col_end = min(col_range*(i+1)+1,col_range);
            for(int r=0;r<rows-1;r++)
            { 
                for(int c=col_start;c<col_end-1;c++)
                {
                    *emask.ptr(r,c) = 255;
                }
            }
            masks[i] = emask.clone();
        }
        pthread_barrier_wait(&displaybarrier);
        
    }
    if (itr != 0)
    {
        //skip if frame 0
        //wait for frames to be done with sobel
        //copy and recombine previous sobel images
        Mat sout;
        for(int i=0;i<da->n;i++)
        {
            da->tout[i].copyTo(sout, masks[i]);
        }
        imshow("Sobel Output",sout);
        waitKey(1);
        printf("DIS: Waiting on display\n");
        pthread_barrier_wait(&displaybarrier);
        //diplay sobel image
    }
    
    //global barrier

    itr += 1;
    //loop
    }
}
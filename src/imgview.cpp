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
 *  Video Demo: 
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
    Mat output_frame; //thread output matrix

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
                .allocated_frame=&allocated_frame,
                .output_frame=&output_frame,
                .cap=&cap
            };
            ret_val[i] = pthread_create(&thread[i], NULL, displayThread, (void *)&da);
        }
        else if(i < thread_count)
        {
            struct sobelArgs *sa = (struct sobelArgs*)malloc(sizeof(struct sobelArgs));
            sa->i=i;
            sa->n=thread_count;
            sa->allocated_frame=&allocated_frame;
            sa->output_frame=&output_frame;

            ret_val[i] = pthread_create(&thread[i], NULL, sobelThread, (void *)sa);
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
    printf("TH%u: entered\n",tn);
    //generate mask
    int itr = 0;
    while(1)
    {
        //wait for global to allocate
        //printf("TH%u: waiting for img allocate\n",tn);
        pthread_barrier_wait(&allocatebarrier);
        //grab frame from allocated_frame

        Mat frame = *sa->allocated_frame;
        Mat emask;
        if (itr == 0)
        {
            int rows = frame.rows;
            int cols = frame.cols;
            int col_range = (cols/sa->n);
            emask = Mat(rows, cols, CV_8U, Scalar(0));
            int col_start = max((col_range*tn), 0);
            int col_end = min((col_range*(tn+1)),cols);
            for(int r=0;r<rows-1;r++)
            { 
                for(int c=col_start;c<col_end-1;c++)
                {
                    *emask.ptr(r,c) = 255;
                }
            }
        }
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
        // printf("TH%i: waiting for display\n",tn);
        pthread_barrier_wait(&displaybarrier);
        //wait for previous frame to be output before copying
        pthread_mutex_lock(&mutex);
        add(sob,*sa->output_frame,*sa->output_frame);
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
    while(1)
    {
    //allocate frame
    *da->cap >> *da->allocated_frame;
    printf("DIS: Allocating...\n");

    //alocate empty output

    pthread_barrier_wait(&allocatebarrier); //wait for all frames to be ready to grab
    //exit if empty frame
    if((*da->allocated_frame).empty()) return 0;
    
    if(itr == 0)
    {
        // printf("DIS: Initializing Masks...\n");
        cols = (*da->allocated_frame).cols;
        rows = (*da->allocated_frame).rows;
        Mat newout (rows, cols, CV_8U, Scalar(0));
        newout.copyTo(*da->output_frame);
        (*da->output_frame) = Scalar(0);
        pthread_barrier_wait(&displaybarrier);
    }
    if (itr != 0)
    {
        //skip if frame 0
        //wait for frames to be done with sobel
        //copy and recombine previous sobel images
        Mat sout(rows, cols, CV_8U, Scalar(0));;

        pthread_mutex_lock(&mutex);
        (*da->output_frame).copyTo(sout);
        pthread_mutex_unlock(&mutex);

        imshow("Sobel Output",sout);
        waitKey(0);
        (*da->output_frame) = Scalar(0);
        // printf("DIS: Waiting on display\n");
        pthread_barrier_wait(&displaybarrier);
        //diplay sobel image
    }
    
    //global barrier

    itr += 1;
    //loop
    }
}
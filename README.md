# Image Viewer
### CPE 442 Project
### Matthew Wong - W24

This project documents the running development of a Sobel filter using the OpenCV library throughout the course of CPE 442.

Usage: `./imgview file_name thread_count debug_code (optional)`

## Test Files
There are two primary test files used - video1 and RoTS-Opening. video1 is relatively small, 400x600 PAL format, while RoTS-Opening is a longer, HD video.

# Lab Development

## Lab 1 - Image Viewer
- Basic image file access

## Lab 2 - Video Viewer
- Basic video playback

## Lab 3 - Sobel filter
- Implimentation of Sobel filter and grayscale conversion

## Lab 4 - Multithreading
- Implimentation of variable multithreading.
- Implimentation of time based performance metric to better quantify improvements.

## Lab 6 - FFMPEG
Command to stream video, no processing
```bash
ffmpeg -re -i video1-crop.mp4 -vcodec h264 -crf 45 -f mpegts 'udp://10.8.0.6:1234?ttl=13'
```
Command to stream video from imgview
```bash
./imgview video1-crop.mp4 4 | ffmpeg -re -f rawvideo -s 800x600 -pix_fmt gray -hwaccel vulkan -i - -c copy -vcodec h264 -crf 42 -f mpegts 'udp://10.8.0.6:1234?ttl=13' 
```
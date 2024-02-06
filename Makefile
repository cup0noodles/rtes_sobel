# CPE 442 - Opencv Viewer Make File
# Matthew Wong - 1/18/24
# 

CC = g++
LIBS = -lpcap -lpthread
CFLAGS = -g -Wall -o0 -Iinc/ -I/usr/include/opencv4/ `pkg-config --libs opencv4`
SRCS = $(wildcard src/*.cpp)
#CFLAGS = -g

all:  default

default:
	$(CC) $(SRCS) $(CFLAGS) -o imgview

clean:
	rm -f imgview

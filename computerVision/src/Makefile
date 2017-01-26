CXX ?= g++ -std=c++11

CXXFLAGS += -c -Wall $(shell pkg-config --cflags opencv)
LDFLAGS += $(shell pkg-config --libs --static opencv)

INC_DIR = -I/usr/local/include/opencv2 \
	  -I/home/pi/opencv-3.2.0/build/include 

all: ../bin/detect_mod

../bin/detect_mod: detect_mod.o; $(CXX) $< -o $@ $(LDFLAGS)

detect_mod.o: detect_mod.cpp; $(CXX) $(INC_DIR) $< -o $@ $(CXXFLAGS) 

clean: ; rm -f detect_markers.o ../bin/detect 

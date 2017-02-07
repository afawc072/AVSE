CXX ?= g++ -std=c++11

CXXFLAGS += -c -Wall $(shell pkg-config --cflags opencv)
LDFLAGS += $(shell pkg-config --libs --static opencv)

INC_DIR = -I/usr/local/include/opencv2 \
	  -I/home/pi/opencv-3.2.0/build/include \
	  -Iinclude

all: bin/computervision

bin/computervision: obj/computervision.o; $(CXX) $< -o $@ $(LDFLAGS)

obj/computervision.o: src/computervision.cpp; $(CXX) $(INC_DIR) $< -o $@ $(CXXFLAGS)

clean: ; rm -f obj/computervision.o bin/computervision


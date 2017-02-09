#
#  Makefile for the Navigation System
#
#  Authors: Alexandre Fawcett, Jean-Sebastien Fiset
#

OBJECT_NAME=AVSE

CPP_SOURCES=$(NAV_SRC_DIR)/Grid.cpp \
	   $(NAV_SRC_DIR)/NavSystemCommon.cpp \
	   $(NAV_SRC_DIR)/NavigationModel.cpp \
	   $(PROT_SRC_DIR)/protocol.cpp \
	   $(SRC_DIR)/AVSE.cpp


SRC_DIR = src
COMP_SRC_DIR = computerVision/src
NAV_SRC_DIR = navigationSystem/src
PROT_SRC_DIR = protocol/src

INC_DIR=-I include \
	-I navigationSystem/include \
	-I protocol/include #	\
#	-I computerVision/include \
#	-I /usr/local/include/opencv2 \
#	-I /home/pi/opencv-3.2.0/build/include

OBJ_DIR= obj
BIN_DIR= bin

TARGET_EXE = $(BIN_DIR)/$(OBJECT_NAME)

CC=g++
#LDFLAG=-L/usr/lib/x86_64-linux-gnu -L/usr/lib64 -L$(LIB_DIR)/linux-x86_64 -L$(LIB_DIR)
#LIBS= -lARgsub_lite -lARvideo -lAR -lARICP -lAR -lglut -lGLU -lGL -lX11 -lm -lpthread -ljpeg
#CFLAG= -O3 -fPIC -march=core2 -DHAVE_NFT=1 -I/usr/include/x86_64-linux-gnu -I$(INC_DIR)

CPPFLAGS = -std=c++11 -g -Wall -I. $(INC_DIR)
OBJS =$(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(CPP_SOURCES))

all: $(TARGET_EXE)

$(OBJ_DIR)/%.o :  $(SRC_DIR)/%.cpp
	mkdir -p $(OBJ_DIR)
	$(CC) -MMD -MP $(CPPFLAGS) -o $@ -c $<

$(TARGET_EXE): $(OBJS)
	mkdir -p $(BIN_DIR)
	$(CC) $(CPPFLAGS) -o $(TARGET_EXE) $(OBJS)


clean:
	rm -f $(OBJ_DIR)/*.o
	rm -f $(TARGET_EXE)
	rm -f $(OBJ_DIR)/*.d
#
#  Makefile for the Navigation System
#
#  Authors: Alexandre Fawcett, Jean-Sebastien Fiset
#

OBJECT_NAME=AVSE

MAIN_CPP = $(SRC_DIR)/AVSE.cpp

OBJS = 	$(NAV_OBJ_DIR)/Grid.o \
	$(NAV_OBJ_DIR)/NavSystemCommon.o \
	$(NAV_OBJ_DIR)/NavigationModel.o \
	$(PROT_OBJ_DIR)/protocol.o \
	$(CV_OBJ_DIR)/computervision.o

SRC_DIR = src
CV_OBJ_DIR = computerVision/obj
NAV_OBJ_DIR = navigationSystem/obj
PROT_OBJ_DIR = protocol/obj

INC_DIR=-I include \
	-I navigationSystem/include \
	-I protocol/include 	\
	-I computerVision/include 


LIBS = $(shell pkg-config --libs --cflags opencv)

OBJ_DIR= obj
BIN_DIR= bin

TARGET_EXE = $(BIN_DIR)/$(OBJECT_NAME)

CC=g++
CPPFLAGS = -std=c++11 

main:
	$(CC) $(CPPFLAGS) $(INC_DIR) $(LIBS) $(MAIN_CPP) $(OBJS) -o $(TARGET_EXE)

all:
	$(MAKE) -C computerVision
	$(MAKE) -C navigationSystem
	$(MAKE) -C protocol

	$(CC) $(CPPFLAGS) $(INC_DIR) $(LIBS) $(MAIN_CPP) $(OBJS) -o $(TARGET_EXE)

navSystem:
	$(MAKE) -C navigationSystem clean
	$(MAKE) -C navigationSystem
	
computerVision:
	$(MAKE) -C computerVision clean
	$(MAKE) -C computerVision

protocol:
	$(MAKE) -C protocol clean
	$(MAKE) -C protocol

clean:
	rm -f $(TARGET_EXE)
	$(MAKE) -C computerVision clean
	$(MAKE) -C navigationSystem clean
	$(MAKE) -C protocol clean

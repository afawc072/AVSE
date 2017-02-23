#
#  Makefile for the Navigation System
#
#  Author(s): Alexandre Fawcett, Jean-Sebastien Fiset
#

OBJECT_NAME=protocol

CPP_SOURCES=$(SRC_DIR)/test.cpp \
	$(SRC_DIR)/protocol.cpp

SRC_DIR=src
INC_DIR=-I include 
OBJ_DIR= obj
BIN_DIR= bin

TARGET_EXE = $(BIN_DIR)/$(OBJECT_NAME)

CC=g++

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


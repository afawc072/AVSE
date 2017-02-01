/*******************************************************************************
*
* PROJET: AUTONOMOUS VEHICULE IN A STRUCTURED ENVIRONMENT
*
* SECTION: Protocol
*
* AUTHORS: Jean-Sebastien Fiset and Alexandre Fawcett
*
* DESCRIPTION:
*
*	Protocol header file
*
* NOTES:
*
*
*
********************************************************************************/
#ifndef PROTOCOL
#define PROTOCOL

#include <unistd.h>
#include <fcntl.h>
#include <unistd.h> // sleep()
#include <stdio.h>  // fopen(), fclose(), fprintf(), perror()
#include <stdlib.h> // exit() and EXIT_FAILURE
#include <iostream>
#include <sys/ioctl.h>
#include <termios.h>
#include <string>
#include <cstring> // for strncpy
#include <vector>

// IMPORTANT SETTINGS
static const int NB_BYTES = 32;//Defined Buffer Size

static const int SLEEP_S = 250;//Short Sleep(ms);
static const int SLEEP_M = 500;//Medium Sleep(ms);
static const int SLEEP_L = 1000;//Long Sleep(ms);

static const char HEADER_START = '[';//Header Start;
static const char HEADER_END = ']';//Header Stop;
static const char HEADER_SPACE = ':';//Header Separation;

using namespace std;

//PROTOCOL DICTIONNARY
static  vector<string> PROTOCOL_DICT={"READY",
					"LISTEN",
					"RCVD",
					"END",
					"WAIT",
					"CAMANGLE",
					"NEXTVEC",
					"REACHED",
					"ADDOBS",
					"STOP",
					"ERROR",};

typedef enum
{
	READY,
	LISTEN,
	RCVD,
	END,
	WAIT,
	CAMANGLE
	NEXTVEC,
	REACHED,
	ADDOBS,
	STOP,
	ERROR,
}command;

static  vector<string> PROTOCOL_ERR={"ERRORTC",
					"ERRORCL",
					"ERRORINI",
					"ERRORWP",};

typedef enum
{
	ERRORTC,//testConnection Error(Listen was not received).
  ERRORCL,//Close Error;
  ERRORINI,//Init Error;
  ERRORWP,//ERROR by using function write(returned -1);

}errorType;

class Protocol
{
public:
	Protocol();
	bool init(errorType &apE);
  bool testConnection(errorType &apE);
	bool send(command aCommand, string aInfoW, errorType &apE);
  bool receive(command &apCommand, string &apInfoR, errorType &apE );
  bool closeP(errorType &apE);

private:

	int fd;

	bool writeP(command aCommand, string aInfoW, errorType &apE);

	bool readP(command &apCommand, string &apInfoR, errorType &apE);
};

#endif /*PROTOCOL*/

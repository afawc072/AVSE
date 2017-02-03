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
#include <sstream> //ostringstream
#include <vector>

// IMPORTANT SETTINGS
static const int NB_BYTES = 34;//Defined Buffer Size

static const int SLEEP_S = 1;//Short Sleep(ms);
static const int SLEEP_M = 1.5;//Medium Sleep(ms);
static const int SLEEP_L = 2;//Long Sleep(ms);

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
					"NEXT",
					"REACHED",
					"ADDOBS",
					"STOP",
					"ERROR",};

typedef enum
{
	READY, //command send by the function testConnection to the arduino
	LISTEN,//expected command received by the arduino in testConnection.
	RCVD,
	END,
	WAIT,
	CAMANGLE,
	NEXT,
	REACHED,
	ADDOBS,
	STOP,
	ERROR,
}command;

static  vector<string> PROTOCOL_ERR={"ERRORTC",
					"ERRORCL",
					"ERRORINI",
					"ERRORWP",
          "ERRORR",
          "ERRORH",
          "ERRORS",};

typedef enum
{
	ERRORTC,//testConnection Error(Listen was not received).
  ERRORCL,//Close Error;
  ERRORINI,//Init Error;
  ERRORWP,//ERROR by using function write(returned -1);
  ERRORR,//readP error:
  ERRORH,//
  ERRORS,

}errorType;

class Protocol
{
public:
	Protocol();
	bool init(errorType &apE);
  bool testConnection(errorType &apE);
	bool send(command aCommand, string aInfoW, errorType &apE);
  bool receive(int aNumAttempts, int aDelay, command &apCommand, string &apInfoR, errorType &apE );
  bool closeP(errorType &apE);


private:

	int fd;

	bool writeP(command aCommand, string aInfoW, errorType &apE);

	bool readP(command &apCommand, string &apInfoR, errorType &apE);

  bool findCommand(string aCmd, command &arCmd);
};

#endif /*PROTOCOL*/

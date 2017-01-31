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
static const int NB_BYTES = 16;//Defined Buffer Size

using namespace std;

//PROTOCOL DICTIONNARY
static  vector<string> PROTOCOL_DICT={"READY",
					"LISTEN",
					"RCVD",
					"END",
					"WAIT",
					"STARTCAM",
					"STOPCAM",
					"NEXTVEC",
					"REACHED",
					"ADDOBS",
					"open",
                                        "STOP"};

typedef enum
{
	READY,
	LISTEN,
	RCVD,
	END,
	WAIT,
	STARTCAM,
	STOPCAM,
	NEXTVEC,
	REACHED,
	ADDOBS,
	OPEN,
	STOP,
}command;


class Protocol
{
public:
	Protocol();
	int init();
	bool send(command aCommand, string aInfoW  );

	 bool receive(command *apCommand, string *apInfoR );

private:

	int fd;

	bool writeP(command aCommand, string aInfoW  );

	bool readP(command *apCommand, string *apInfoR );
};

#endif /*PROTOCOL*/

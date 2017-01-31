/*******************************************************************************
*
* PROJET: AUTONOMOUS VEHICULE IN A STRUCTURED ENVIRONMENT
*
* SECTION: Protocol
*
* AUTHORS: Alexandre Fawcett and Jean-Sebastien Fiset
*
* DESCRIPTION:
*
*	Protocol function for communication between a Raspberry Pi and Arduino
*	over serial USB
*
* NOTES:
*
*
*
********************************************************************************/


#include "protocol.h"
Protocol::Protocol()
{
}

int Protocol::init()
{
	fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
	cout << "3 second sleep" << endl;
	usleep(3000);
	return 0;	
}


/******************************************************************************* 
 * Send
 *
 * 	Function to be by other c++ programs to send data to arduino
 *
 *******************************************************************************/
bool Protocol::send(command aCommand, string aInfoW  )
{
	string temp = PROTOCOL_DICT[aCommand];
	usleep(5000);
	cout << temp << endl;
	cout << "NIGGA";
	char bufw[16];
	cout << "STR TO COPY BEFORE";
	strncpy(bufw, temp.c_str(), sizeof(bufw));
	bufw[sizeof(bufw) - 1] = 0;
	cout << "AFTER STR@CPY";
	char bufr[NB_BYTES];
 	cout << "before write";
	write(fd, bufw, 16);
	usleep(1000);
	cout << "after sleep";
	int n = read(fd, bufr, NB_BYTES);

	return true;
}

/*******************************************************************************
 * Send
 *
 *      Function to be by other c++ programs to send data to arduino
 *
 *******************************************************************************/
bool Protocol::receive(command *apCommand, string *apInfoR )
{
read(fd,apCommand,NB_BYTES);
return true;
}
/*******************************************************************************
 * Send
 *
 *      Function to be by other c++ programs to send data to arduino
 *
 *******************************************************************************/
bool Protocol::writeP(command aCommand, string aInfoW  ){
return true;
}

/*******************************************************************************
 * Send
 *
 *      Function to be by other c++ programs to send data to arduino
 *
 *******************************************************************************/
bool Protocol::readP(command *apCommand, string *apInfoR )
{
return true;
}

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
*	Base Idea for the way to interact with the ttyACM0: https://chrisheydrick.com/2012/06/24/how-to-read-serial-data-from-an-arduino-in-linux-with-c-part-4/
*
*
*********************************************************************************/


#include "protocol.h"
Protocol::Protocol()
{
}

bool Protocol::init(errorType &apE)
{
  //Open the File Directory /dev/ttyACM0 (Arduino)
  fd=open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NONBLOCK);
  //This is done twice because we had issues...
  fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NONBLOCK);

  //ADD Error Check;

  return true;
}

/*******************************************************************************
 * testConnection
 *
 * 	Function to ensure that the arduino is ready to receive data
 *
 *******************************************************************************/
bool Protocol::testConnection(errorType &apE)
{
  //returned value by this function and received by write
  bool flagTC=false;
  //READY command for connectionTesting
  command cmd = READY;
  // wWill act as a pointer for the received command by readP
  command rcv;
  string nUll="";
  /*
  The following logic sends the ready command to the arduino, and in the case
  the write is successful, we receive LISTEN, hopefully.
  */
  if(writeP(cmd,nUll,apE))
  {
    //Sleep Function to ensure proper synchronization?!?
    usleep(SLEEP_S);
    if(readP(rcv,nUll,apE))
    {
      //The connection test shoudl return LISTEN by the ARDUINO.
      if(rcv==LISTEN)
      {
        flagTC=true;
      }
      else
      {
       apE=ERRORTC;
      }
    }
  }
  return flagTC;
}


/*******************************************************************************
 * Send
 *
 * 	Function to be by other c++ programs to send data to arduino
 *
 *******************************************************************************/
bool Protocol::send(command aCommand, string aInfoW, errorType &apE )
{
  bool flagS=false;
  /*
  In the following logic, we test connection,
  then write to the arduino what we need.
  */
  if(testConnection(apE))
  {
    if(writeP(aCommand, aInfoW, apE))
    {
      flagS=true;
    }
  }
	return flagS;
}

/*******************************************************************************
 * Receive
 *
 *      Function to be by other c++ programs to receive data from arduino
 *
 *******************************************************************************/
bool Protocol::receive(command& apCommand, string &apInfoR, errorType &apE)
{
  bool flagR=false;

  if(readP(apCommand, apInfoR, apE))
  {
    flagR=true;
  }
  return flagR;
}
/*******************************************************************************
 * WriteP
 *
 *      Function to be by other c++ programs to send data to arduino
 *
 *******************************************************************************/
bool Protocol::writeP(command aCommand, string aInfoW, errorType &apE)
{
  //Define the command received as a String
  string cmd = PROTOCOL_DICT[aCommand];
  //Size of the command string
  int sizeCmd = sizeof(cmd);
  int sizeInfo;
  //flag for write error
  bool flagE=false;
  bool flagW=false;

  //flag for the final char array building
  //The buffer that will contain the sendable data as a char Array.
  char bufw[NB_BYTES];
  char bufTempCmd[sizeCmd];
  char* bufTempInfo;

  //Transform the string to a char[]
  strncpy(bufTempCmd, cmd.c_str(), sizeCmd);

  //Check if Info is NULL and create its char array if non-empty.
  if(aInfoW!="")
  {
    flagE=true;
    sizeInfo = sizeof(aInfoW);
    bufTempInfo= (char*) malloc(sizeInfo);
    strncpy(bufTempInfo, aInfoW.c_str(), sizeInfo);
  }

  /*
  Loop to fill the bufw with the information of the command and aInfoW
  */
  //header
  bufw[0]=HEADER_START;
  int i;
  for(i=0;i<=sizeCmd;i++)
  {
    bufw[(i+1)]=bufTempCmd[i];
  }

  bufw[i]=HEADER_SPACE;

  if(flagE)
  {
    int j=0;
    for(j=0;j<sizeInfo;j++,i++)
    {
      bufw[(i+1)]=bufTempInfo[j];
    }
  }

  bufw[i]=HEADER_END;

	int w=write(fd, bufw, NB_BYTES);
  //ERROR RETURN
  if(w!=-1)
  {
    flagW=true;
  }
  else if(w==-1)
  {
    apE=ERRORWP;
  }
  else
  {
  }

  return flagW;
}

/*******************************************************************************
 * ReadP
 *
 *      Function to be by other c++ programs to send data to arduino
 *
 *******************************************************************************/
bool Protocol::readP(command &apCommand, string &apInfoR, errorType &apE)
{
  return true;
}

/*******************************************************************************
 * CloseP
 *
 *      Function to close the file descriptor
 *
 *******************************************************************************/
bool Protocol::closeP(errorType &apE)
{
  close(fd);//close the file descriptor
  if(close(fd)!=-1)
  {
    return true;
  }
  else
  {
    apE=ERRORCL;
    return false;
  }
}

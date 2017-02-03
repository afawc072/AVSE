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

/*******************************************************************************
 * init
 *
 * 	Open the connection to the arduino by opening a file descriptor to it.
 *  A bash code executing a python file is executed beforehand to lock
 *  the proper serial parameters in the Pi in order to read the data.
 *
 *******************************************************************************/
bool Protocol::init(errorType &apE)
{
  //System call to run the bash script executing the python code.
  system("bin/pybaud.sh");

  struct termios toptions;

  /*
  Open the File Directory /dev/ttyACM0 (Arduino). The file descriptor "fd" has 3 settings:
  O_RDWR: Open for reading and writing
  O_NOCTTY: The terminal doesn't become the controlling device
  O_NONBLOCK: Please read further @ http://pubs.opengroup.org/onlinepubs/009695399/functions/open.html
  */
  fd=open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NONBLOCK);

  //Error Check: if the file can't be opened, -1 is set to fd.
  if(fd==-1)
  {
    apE=ERRORINI;
    return false;
  }
  return true;
}

/*******************************************************************************
 * testConnection
 *
 * 	Function to ensure that the arduino is ready to receive data
 *  The command READY is sent and we expect to read LISTEN.
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
    sleep(SLEEP_S);
    //TROUBLESHOOTING by printing to the terminal.
    cout << "WRITEP OK" << endl;

    //Condition if readP is done.
    if(readP(rcv,nUll,apE))
    {
      //Sleep to allow the arduino to write back to the serial when READY is sent
      sleep(SLEEP_S);
      //The connection test shoudl return LISTEN by the ARDUINO.
      cout << "READP OK" << endl;
      if(rcv==LISTEN)
      {
        flagTC=true;
      }
      else
      {
       apE=ERRORTC; //the reference to the error is set ERRORTC.
      }
    }
  }
  return flagTC;
}


/*******************************************************************************
 * Send
 *
 * 	Function to be called by other c++ programs to send data to arduino
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
bool Protocol::receive(int aNumAttempts, int aDelay, command& apCommand, string &apInfoR, errorType &apE)
{
  bool flagR=false;
  int counter = 0;

  /*
  Loop to be executed when receiving information to wait for a successful read.
  each loop is characterized by a delay defined by the program calling this function, including
  the number of loop to be executed.
  */
  while( counter < aNumAttempts && !flagR)
  {
     if(readP(apCommand, apInfoR, apE))
     {
       flagR=true;
     }
     counter++;
     sleep(aDelay);
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
  int sizeCmd = cmd.length();
  int sizeInfo;
  //flag for write error
  bool flagE=false;
  bool flagW=false;

  //flag for the final char array building
  //The buffer that will contain the sendable data as a char Array.
  char bufw[NB_BYTES];
  char bufTempCmd[sizeCmd];
  char* bufTempInfo;

  //Fill in the buffers array with 0's so that nill array postions are't filled with non-defined encoding symbols.

  fill_n(bufw,NB_BYTES,0);
  fill_n(bufw,NB_BYTES,0);
  //Transform the string to a char[]. Size cmd increase because % appears (No explanation).
  strncpy(bufTempCmd, cmd.c_str(), (sizeCmd+1));
  //Check if Info is NULL and create its char array if non-empty.
  if(aInfoW!="")
  {
    //flag set for further use(to send the info also).
    flagE=true;
    sizeInfo = aInfoW.length();
    bufTempInfo= new char[sizeInfo];
    strncpy(bufTempInfo, aInfoW.c_str(), sizeInfo);
  }

  /*
  Loop to fill the bufw with the information of the command and aInfoW
  */
  //header
  bufw[0]=HEADER_START;
  int i;
  for(i=0;i<sizeCmd;i++)
  {
    bufw[(i+1)]=bufTempCmd[i];
  }
 i++;
  bufw[i]=HEADER_SPACE;
  if(flagE)
  {
    int j=0;
    for(j=0;j<sizeInfo;j++)
    {
      bufw[(i+1)]=bufTempInfo[j];
      i++;
    }
  }
  i++;
  bufw[i]=HEADER_END;
  cout << "BUFW " << bufw << endl;
  sleep(SLEEP_L);
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
  bool flagR =true;
  char bufr[NB_BYTES];
  char bufCmd[NB_BYTES];
  char bufInfo[NB_BYTES];

  fill_n(bufr,NB_BYTES,0);
  fill_n(bufCmd,NB_BYTES,0);
  fill_n(bufInfo,NB_BYTES,0);

  int ret;
  int i=0;
  int j=0;

  string cmd;
  string info;
//troubleshoot
  sleep(SLEEP_S);
  ret=read(fd,bufr,NB_BYTES);
  cout <<"BUFR " << bufr << endl;

  //error for read
  if(ret==-1)
  {
    apE=ERRORR;
    return false;
  }
  else
  {
    //Parse the string;
    if(bufr[i]!=HEADER_START){
      apE=ERRORH;
      return false;
    }
    else
    {
      i++;
      j=0;
      while(bufr[i]!=HEADER_SPACE)
      {
        if(i==(NB_BYTES-1))
        {
          apE=ERRORS;
          return false;
        }
          bufCmd[j]=bufr[i];
	j++;
          i++;
      }
      i++;
      cmd=string(bufCmd);

      flagR=findCommand(cmd, apCommand);

      if(!flagR)
      {
        apE=ERRORS;
        cout << "FIND COMMAND ERROR " << endl;
        return false;
      }
      else if(bufr[i]==HEADER_END)
      {
        return flagR;
      }
      else
      {
        j=0;
        while(bufr[i]!=HEADER_END)
        {
          if(i== sizeof(bufr))
          {
            cout << i<<" NO HEADER_END " << endl;
            apE=ERRORS;
            return false;
          }
            bufInfo[j]=bufr[i];
            j++;
            i++;
        }
        apInfoR=string(bufInfo);
        return flagR;
      }

    }

  }
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

bool Protocol::findCommand(string aCmd, command &arCmd)
{
   bool validCmd = false;

   for( unsigned int ix = 0 ; ix < PROTOCOL_DICT.size(); ix++ )
   {
      if( aCmd == PROTOCOL_DICT[ix] )
      {
         arCmd = (command) ix;
         validCmd = true;
      }
   }
   return validCmd;
}



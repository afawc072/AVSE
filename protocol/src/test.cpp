#include <protocol.h>

int main(){

  Protocol test;
  errorType error;
  string errorS;
  string sendS;
  string receiveS;
  cout << "INIT TEST" << endl;
  //cout.flush();
  sleep(2);

  if(!test.init(error))
  {
    errorS=PROTOCOL_ERR[error];
    cout << errorS << endl;
    //cout.flush();
  }
  cout << "SEND ANGLE" << endl;
  sleep(2);
  sendS="5";

  if(!test.send(CAMANGLE,sendS,error))
  {
    errorS = PROTOCOL_ERR[error];
    cout << errorS << endl;
  }
 
 cout << "READING.." << endl;
  //cout.flush();

  command rCmd;
  string receivedInfo;
  int n = 5;
  int delay = SLEEP_S;;
  if(!test.receive(n,delay,rCmd,receivedInfo,error))
  {
    cout << PROTOCOL_ERR[error]<<endl;
  }
  else
  {
    cout<< "RECEIVED CMD: "<< PROTOCOL_DICT[rCmd]<<endl;
  }

  sleep(2);
  cout << "SENDING NEXT"<<endl;
  float x = 7.07;
  float y = 7.07;
  ostringstream infoNext;
  infoNext << x<<","<<y;
  string info = infoNext.str();
   if(!test.send(NEXT,info,error))
  {
    errorS = PROTOCOL_ERR[error];
    cout << errorS << endl;
  }

 
   cout << "READING.." << endl;
  //cout.flush();


  if(!test.receive(10,SLEEP_L,rCmd,receivedInfo,error))
  {
    cout << PROTOCOL_ERR[error]<<endl;
  }
  else
  {
    cout<< "RECEIVED CMD: "<< PROTOCOL_DICT[rCmd]<<endl;
  }

/*
command rCommand;
string rInfo;
if(!test.receive(&rCommand, &rInfo))
{
cout << "error" << endl;
}
//use rCommand and rInfo here
*/

}

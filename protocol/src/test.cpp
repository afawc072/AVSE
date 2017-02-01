#include <protocol.h>

int main(){

  Protocol test;
  errorType error;
  string errorS;
  string sendS;
  string receiveS;
  cout << "INIT TEST" << endl;
  sleep(3000);

  if(!test.init(error))
  {
    errorS=PROTOCOL_ERR[error];
    cout << errorS << endl;
  }
  cout << "SEND ANGLE" << endl;
  sleep(3000);
  sendS="5";
  if(!test.send(CAMANGLE,sendS,error))
  {
    errorS = PROTOCOL_ERR[error];
    cout << errorS << endl;
  }
  cout << "RECEIVE CONFG" << endl;
  sleep(3000);

  //if(!test.receive(CAMANGLE,receiveS,error))
  //{
  //  errorS = PROTOCOL_DICT[aCommand];
  //  cout << errorS << endl;
  //}

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

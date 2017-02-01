#include <protocol.h>

int main(){

   Protocol test;
   errorType error;

   if(!test.init(error))
   {
      // error
   }

   cout << "waiting..." << endl;

   if(!test.send(OPEN,"",error))
   {
      // error
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

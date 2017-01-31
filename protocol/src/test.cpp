#include <protocol.h>

int main(){

Protocol test;
test.init();
cout << "waiting..." << endl;

bool t = test.send(OPEN,"");
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

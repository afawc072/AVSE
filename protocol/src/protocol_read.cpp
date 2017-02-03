#include <unistd.h>
#include <fcntl.h>    
#include <unistd.h> // sleep()
#include <stdio.h>  // fopen(), fclose(), fprintf(), perror()
#include <stdlib.h> // exit() and EXIT_FAILURE
#include <iostream>
//#include <string>
#include <sys/ioctl.h>
#include <termios.h>
using namespace std;

int main() {
    int n, fd;
    //FILE *file;

 
      while(true)
    {
	char bufr[16];
	char bufw[16];
	fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
	usleep(100);
	cout << "open or close" << endl;
	cin >> bufw;
	write(fd, bufw, 16);
	n = read(fd, bufr, 16);
 /* insert terminating zero in the string */ 
	//bufr[n] = 0;
	cout << n << " bytes "<< bufr << endl;
	//printf("%i bytes read, buffer contains: %s \n", n, bufr);        
        //if( 0 >= fprintf(file,"%i",input) ) //Writing to the file
        //{ // fprintf failed
        //    perror("fprintf data failed" );
        //    exit( EXIT_FAILURE );
        //}
	//fclose(file);

     }


    //return 0;
}

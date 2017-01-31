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
   // char input;
   // char output;
    int n, fd;
    //FILE *file;

 
      while(true)
    {
	char bufr[16];
	char bufw[16];
	fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
	//file = fopen("/dev/ttyACM0", "w");
	//cout << "Please put in 0 or 1" << endl;
        //cin >> input;
        //cout << "You've entered " << input << endl;
        //fprintf(file,"%c",input);
	//fclose(file);
	//file = fopen("/dev/ttyACM0", "r+");
	 /* Receive string from Arduino */
	usleep(3500);
	//read(fd,bufr,64);
	cout << "open or close" << endl;
	cin >> bufw;
	write(fd, bufw, 16);
	//usleep(1000); 
	n = read(fd, bufr, 16);
 /* insert terminating zero in the string */
 	//bufr[n] = 0;
 	tcflush(fd,TCIOFLUSH);
	
	cout << n << " bytes "<< bufr << endl;
	//printf("%i bytes read, buffer contains: %s \n", n, bufr);        
	//cout << output;
        //if( 0 >= fprintf(file,"%i",input) ) //Writing to the file
        //{ // fprintf failed
        //    perror("fprintf data failed" );
        //    exit( EXIT_FAILURE );
        //}
	//fclose(file);

     }


    //write(fd, "X", 1);
    //ssize_t size = read(fd, &byte, 1);
    //printf("Read byte %c\n", byte);
    //return 0;
}

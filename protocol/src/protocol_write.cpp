#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>    
#include <unistd.h> // sleep()
#include <stdio.h>  // fopen(), fclose(), fprintf(), perror()
#include <stdlib.h> // exit() and EXIT_FAILURE
#include <iostream>

using namespace std;

int main() {
    char input;
    FILE *file;

 
      while(true)
    {
	file = fopen("/dev/ttyACM0", "w");
        cout << "Please put in 0 or 1" << endl;
        cin >> input;
        cout << "You've entered " << input << endl;
        fprintf(file,"%c",input);
	fclose(file);
        //if( 0 >= fprintf(file,"%i",input) ) //Writing to the file
        //{ // fprintf failed
        //    perror("fprintf data failed" );
        //    exit( EXIT_FAILURE );
        //}

     }


    //write(fd, "X", 1);
    //ssize_t size = read(fd, &byte, 1);
    //printf("Read byte %c\n", byte);
    //return 0;
}

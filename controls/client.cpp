// client.cpp
// created 1-27-13
// -> translation of client.py into cpp

#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <sys/socket.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include "aux_socket.h"

using namespace std;


// function taken from http://cboard.cprogramming.com/c-programming/63166-kbhit-linux.html
int kbhit(void)
{
  struct termios oldt, newt;
    int ch;
	  int oldf;
	   
	     tcgetattr(STDIN_FILENO, &oldt);
		   newt = oldt;
		     newt.c_lflag &= ~(ICANON | ECHO);
			   tcsetattr(STDIN_FILENO, TCSANOW, &newt);
			     oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
				   fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
				    
					  ch = getchar();
					   
					     tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
						   fcntl(STDIN_FILENO, F_SETFL, oldf);
						    
							  if(ch != EOF)
							    {
								    ungetc(ch, stdin);
									    return 1;
										  }
										   
										     return 0;
											 }

int main(int argc, char **argv){
	
	if(argc < 3){
		perror("Input should be of the form '[program] [host] [port]'\n");
		exit(1);
	}

	char c;
	int csocket = newClientSocket(argv[1], argv[2]);

	while(true){
		// read and send data to server
		
		// old - using input stream
		// scanf("%c", &c);

		// using keypresses
		if(kbhit()){
			c = getchar();

			switch(c){
				case 65:	// up arrow
					c = 'u';
					break;
				case 66:	// down arrow
					c = 'd';
					break;
				case 67:	// right arrow
					c = 'r';
					break;
				case 68:	// left arrow
					c = 'l';
					break;
				case 's':
					break;
				case 'f':
					break;
				default:
					c = '0';
					break;
			}
			
			// continue with for loop if there was no control input
			if(c == '0'){
				continue;	
			}

			printf("'%c' was pressed\n", c);

			int ret = 0;
			while((ret = send(csocket, &c, csocket, 1)) < 1){
				if(ret == -1)
					perror("Error sending data to server.\n");
			}
		}
	}
}

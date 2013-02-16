// server.cpp
// created 1-27-13
// -> translation of server.py into cpp

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <string>
#include <signal.h>

#include "aux_socket.h"

using namespace std;

int serverfd;

// TODO: put this in a class?
float heading = 0.0;
int vl = 0;
int vr = 0;

int accel = 4;

int ser;
speed_t baud = B9600;
static int SIZEOFINT = 4;
static int CNUM = 128; 	// correction number for sending data to arduino

void connection_handler(int serverfd, int acceptfd){
	char message;
	char garb;	// garbage

	while(true){

		// read & display input
		//
		//	s - Stop		r - Right	u - speed Up
		//	f - Forward		l - Left	d - speed Down
		//
		while (recv(acceptfd, &message, 1, 0) != 1)
			continue;

		// ignore garbage
		//  1 character for keypress
		//	3 characters for keybord input (stdin)
		for(int i = 0; i < 1; i++){
			while(recv(acceptfd, &garb, 1, 0) != 1)
				continue;
		}
		

		printf("Received message: %c\n", message);

		// send input to robot
		int temp;
		switch (message){
		case 's':
			vl = vr = 0;
			break;
		case 'f':
			temp = vl > vr ? vl : vr;	// temp = max
			vr = vl = temp;
			break;
		case 'r':
			temp = vl < vr ? vl : vr;	// temp = min
			vr = 0 - temp;
			vl = temp;
			break;
		case 'l':
			temp = vl < vr ? vl : vr;	// temp = min
			vr = temp;
			vl = 0 - temp;
			break;
		case 'u':
			vr -= accel;
			vl -= accel;
			break;
		case 'd':
			vr += accel;
			vl += accel;
		}
		
		int d1 = vl - CNUM;
		int d2 = vr - CNUM;
		write(ser, &d1, SIZEOFINT);
		write(ser, &d2, SIZEOFINT);
		write(ser, &CNUM, SIZEOFINT);
	}

	return;
}

// handle interrupts:
// -close socket connection
// -exit
void sigINT_handler(int s){
	if(serverfd > -1)
		closeSocket(serverfd);

	close(ser);

	exit(1);
}


// main server function
int main(int argc, char *argv[]){

	string host = "127.0.0.1";
	char port[] = "12345";
	serverfd = -2;
	int acceptfd = -2;
	
	// set up interrupt handler
	signal(2, &sigINT_handler); // 2 = SIGINT

	// set up server listening on port 'port'
	if((serverfd = newServerSocket(port)) == -1){
		perror("server.cpp: serverfd not set up\n");
		exit(1);
	}

	// open port for communication with arduino
	if((ser = open("/dev/ttyACM0", O_RDWR | O_FSYNC | O_TRUNC)) == -1){
		printf("Unable to open port /dev/ttyACM0,\n");
//		exit(1);	TODO: uncomment once connected to arduino
	}else{
		// set baud rate
		struct termios settings;
		tcgetattr(ser, &settings);
		cfsetospeed(&settings, baud);
		tcsetattr(ser, TCSANOW, &settings);
		tcflush(ser, TCOFLUSH);
		
		printf("Port is now open.\n");
	}

	// accept connections
	while(1){
		if((acceptfd = acceptSocket(serverfd)) == -1){
			perror("server.cpp: error accepting connection\n");
			continue;	// drop request
		}
		
		// handle request
		connection_handler(serverfd, acceptfd);
	}
}

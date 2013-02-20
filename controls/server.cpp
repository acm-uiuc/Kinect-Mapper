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

string buf;
size_t BUFSIZE = 100;

int ser;
speed_t baud = B9600;
static char CNUM = 128; 	// correction number for sending data to arduino

void connection_handler(int serverfd, int acceptfd){
	
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
	char port[] = "12342";
	serverfd = -2;
	int acceptfd = -2;
	buf = new char(100);
	struct termios settings;
	
	// set up interrupt handler
	signal(2, &sigINT_handler); // 2 = SIGINT

	// set up server listening on port 'port'
	if((serverfd = newServerSocket(port)) == -1){
		perror("server.cpp: serverfd not set up\n");
		exit(1);
	}

	// open port for communication with arduino
	if((ser = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY)) < 0){
		printf("Unable to open port /dev/ttyACM0\n");
		return -1;
	}
	if( tcgetattr(ser, &settings) < 0){
		printf("init of serial port: Couldn't get ser attributes\n");
		return -1;
	}

	// set baud rate
	cfsetispeed(&settings, baud);
	cfsetospeed(&settings, baud);

	// set char size
	settings.c_cflag &= ~PARENB;
	settings.c_cflag &= ~CSTOPB;
	settings.c_cflag &= ~CSIZE;
	settings.c_cflag |= CS8;

	// turn off software flow control
	settings.c_cflag |= CLOCAL | CREAD;
	settings.c_iflag &= ~(IXON | IXOFF |IXANY);

	// make raw
	settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	settings.c_oflag &= ~OPOST;

	settings.c_cc[VMIN] = 0;
	settings.c_cc[VTIME] = 20;

	if( tcsetattr(ser, TCSANOW, &settings) < 0){
		printf("init of serial port: Couln't set term attribs\n");
		return -1;
	}
		
	printf("Port is now open.\n");

	// test write TODO: delete this
	for(int i = 0; i < 1; i++){
	char c1 = 128;
	char c2 = 128;
	int n = write(ser, &c2, 1);
	if (n != 1)
		printf("Could not write c1\n");
	n = write(ser, &c2, 1);
	if(n!=1)
		printf("Could not write c2\n");
	n = write(ser, &c1, 1);
	if(n != 1)
		printf("Could not write c3\n");
	}
	// accept connections
	while(1){
		if((acceptfd = acceptSocket(serverfd)) == -1){
			perror("server.cpp: error accepting connection\n");
			continue;	// drop request
		}
		
		// handle request
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
			int vel = (vl >= 0 ? vl : 0-vl);
			int temp;
			switch (message){
			case 's':
				vl = vr = 0;
				break;
			case 'f':
				vr = vl = vel > 0 ? vel : accel;
				break;
			case 'r':
				vr = 0 - vel;
				vl = vel;
				break;
			case 'l':
				vr = vel;
				vl = 0 - vel;
				break;
			case 'u':
				vr += vr >= 0 ? accel : 0 - accel;
				vl += vl >= 0 ? accel : 0 - accel;
				break;
			case 'd':
				vr -= vr >= 0 ? 0 - accel : accel;
				vl -= vl >= 0 ? 0 - accel : accel;
				break;
			}
		
			// read line from serial
			/*char b[1];
			int i = 0;
			do{
				int n = read(ser, b, 1);
				if( n == -1) return -1;
				if( n == 0){
					usleep(10000);	// wait 10 ms
					continue;
				}
				buf[i] = b[0];
				i++;
			} while (b[0] != '\n');

			buf[i] = 0;

			read(ser, &buf, BUFSIZE);
			printf("%s", &buf);
			*/
			// write to serial
			char d1 = (char) (vl + (int)CNUM);
			char d2 = (char) (vr + (int)CNUM);
			int err;
			
			if(write(ser, &d1, 1) == 1)
				printf("Ec1");
			if(write(ser, &d2, 1) == 1)
				printf("Ec2");
			if(write(ser, &CNUM, 1) == 1)
				printf("Ec3");			
			
		}

	}
}

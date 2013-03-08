
#include "interface.h"
#include "robot.h"
#include "server.h"
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/socket.h>
#include <stdlib.h>

int Interface::afd = -2;
int Interface::sfd = -2;

Interface::Interface(){
	rob = new Robot();
	port = new char[5];
	port = "12345";

	CNUM = 128;     // correction number for sending data to arduino

//	planner = plan;
}

Interface::~Interface(){
	if(rob != NULL)
		delete rob;
	if(server != NULL)
		delete server;
	if(port != NULL)
		delete port;
}

int Interface::run(){
	server = new Server();
 	// start communication channel with arduino
	setupArduinoConnection();

    // start server 
	sfd = server->setupServer();
	char message;

	while(1){
		printf("Attempting to accept connection.\n");
		while((afd = server->acceptConnection(sfd)) <= 0)	// keep trying
			printf("afd: %i\n", afd);
		printf("afd: %i\n", afd);

		while(1){
			printf("Attempting to get message\n");
			message = server->getMessage(afd);
			sendMessageToRobot(message, MODE_MANUAL);
		}
	}

	return 0;
}


int Interface::sendMessageToRobot(char message, int mode){
	
	int temp;
	int retval = 0;
	switch (message){
	case 's':
		retval = rob->stop();
		break;
	case 'f':
		retval = rob->go_forward(mode);
		break;
	case 'r':
		retval = rob->turn_right(mode);
		break;
	case 'l':
		retval = rob->turn_left(mode);
		break;
	case 'u':
		retval = rob->speed_up(mode);
		break;
	case 'd':
		retval = rob->slow_down(mode);
		break;
	}

	if(retval == 1)
		writeToSerial();
}

void Interface::passCommand(int command, int source){
	int output = 0;
	switch(command){
	case STOP:
		output = rob->stop();
		break;
	case FORWARD:
		output = rob->go_forward(source);
		break;
	case RIGHT:
		output = rob->turn_right(source);
		break;
	case LEFT:
	  output = rob->turn_left(source);
	  break;
	  
	}

	if(output == 1)
		writeToSerial();
}


void Interface::setMode(int mode){
	rob->set_mode(mode);
}


int Interface::writeToSerial(){
	char d1 = (char) (rob->get_vel_left() + (int)CNUM);
	char d2 = (char) (rob->get_vel_right() + (int)CNUM);
	int err;

	int retval = 0;

	if(write(ser, &d1, 1) == 1){
		printf("Ec1");
		retval = -1;
	}
	if(write(ser, &d2, 1) == 1){
		printf("Ec2");
		retval = -1;
	}
	if(write(ser, &CNUM, 1) == 1){
		printf("Ec3");
		retval = -1;
	}

	return retval;
}

int Interface::setupArduinoConnection(){
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
	
	// start reading data from arduino
	
	if(pthread_attr_init(&attr) != 0){
		printf("Error setting thread attributes.\n");
	}
	
	int* s = new int();
	*s = ser;
	if(pthread_create(&thread_id, &attr, (void* (*)(void*))&Interface::arduino_data_reader, (void*)(s)) != 0)
		printf("Error arduino_data_reader thread.\n");

	while(pthread_detach(thread_id) != 0)
		printf("Error detaching thread\n");

	pthread_exit(NULL);
	return 0;
}


// handle information coming in from arduino
void* Interface::arduino_data_reader(void* args){

	int* s = (int*)(args);
	int ser = *s;
	
    char* buf = (char*) malloc(100);
	int BUFSIZE = 100;

	speed_t baud = B9600;
	while(1){
		// read line from serial
		char b[1];
		int i = 0;
		do{
			int n = read(ser, b, 1);
			if( n == -1) break;
			if( n == 0){
				usleep(10000);	// wait 10 ms
				continue;
			}
			buf[i] = b[0];
			i++;
		} while (b[0] != '\n');

		buf[i] = 0;

		read(ser, buf, BUFSIZE);
		printf("%s", buf);
	}
	delete s;

	pthread_exit(NULL);
}

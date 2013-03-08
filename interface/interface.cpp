
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

Interface::Interface(){
	rob = new Robot();
	port = new char[5];
	port = "12345";

	CNUM = 128;     // correction number for sending data to arduino

	server = new Server(rob);
//	planner = plan;
}

Interface::~Interface(){
	delete rob;
	delete server;
	free(buf);
	delete port;
}

int Interface::run(){
 	// start communication channel with arduino
	setupArduinoConnection();

	// start server thread
	pthread_attr_t server_attr;
	pthread_t server_thread_id;

	if(pthread_attr_init(&server_attr) != 0){
		printf("Error setting server_attr thread attributes\n");
		return -1;
	}
	if(pthread_create(&server_thread_id, &server_attr, &runServer, server) != 0){
		printf("Error creating RunServer thread (in Interface)\n");
		return -1;
	}
	while(pthread_detach(server_thread_id) != 0){
		printf("Error detaching thread\n");
		return -1;
	}

	return 0;
}


static void* runServer(void* args){
	Server* server = (Server*) args;
	server->SetupServer();
}

/*
static void* RunPlanner(void* args){
}*/

void Interface::passCommand(int command, int source){
	int output = 0;
	switch(command){
	case STOP:
		output = rob->stop();
		break;
	case FORWARD:
		output = rob->go_forward(source);
		break;
	case NODATA:
		output = rob->turn_right(source);
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
	if((ser = open("/dev/ttyACM1", O_RDWR | O_NOCTTY | O_NDELAY)) < 0){
		printf("Unable to open port /dev/ttyACM1\n");
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
	pthread_attr_t attr;
	if(pthread_attr_init(&attr) != 0){
		printf("Error setting thread attributes.\n");
	}
	pthread_t thread_id;
	if(pthread_create(&thread_id, &attr, &arduino_data_reader, (void*)ser) != 0)
		printf("Error arduino_data_reader thread.\n");

	while(pthread_detach(thread_id) != 0)
		printf("Error detaching thread\n");

	return 0;
}


// handle information coming in from arduino
static void* arduino_data_reader(void* args){

	int ser = (int)(args);
	
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

	return NULL;
}

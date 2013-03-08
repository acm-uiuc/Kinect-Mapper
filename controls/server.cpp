// server.cpp
// created 1-27-13

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
#include "robot.h"
#include "server.h"

using namespace std;

Server::Server(){
  	
	host = "127.0.0.1";
	port = "12342";
	serverfd = -2;
	acceptfd = -2;
}


Server::~Server(){
  	if(serverfd > -1)
    	closeSocket(serverfd);
}


int Server::setupServer(){
	// set up server listening on port 'port'
	if((serverfd = newServerSocket(port)) == -1){
		perror("server.cpp: serverfd not set up\n");
		return -1;
	}
/*	
	printf("Attermpting to accept on %i\n", serverfd);
	if((acceptfd = acceptSocket(serverfd)) == -1){
		printf("server.cpp: error accepting connection\n");
		return -1;	// drop request
	}

	return acceptfd;*/
	return serverfd;
}

int Server::acceptConnection(int sfd){
	printf("Attermpting to accept on %i\n", sfd);
	if((acceptfd = acceptSocket(sfd)) == -1){
		printf("server.cpp: error accepting connection\n");
		return -1;	// drop request
	}
	return acceptfd;
}
		
char Server::getMessage(int acceptfd){
	// handle request
	char message;
	char garb;	// garbage

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

	return message;
}

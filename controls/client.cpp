// client.cpp
// created 1-27-13
// -> translation of client.py into cpp

#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <sys/socket.h>

#include "aux_socket.h"

using namespace std;

int main(int argc, char **argv){
	
	if(argc < 3){
		perror("Input should be of the form '[program] [host] [port]'\n");
		exit(1);
	}

	char imp;
	int csocket = newClientSocket(argv[1], argv[2]);

	while(true){
		// read and send data to server
		scanf("%c", &imp);

		int ret = 0;
		while((ret = send(csocket, &imp, csocket, 1)) < 1){
			if(ret == -1)
				perror("Error sending data to server.\n");
		}
	}
}

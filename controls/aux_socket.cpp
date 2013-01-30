#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>

#include "aux_socket.h"

const int BACKLOG = 100;


/**
 * creates a new socket file descriptor, binds to the local port port,
 * then sets up a listening queue.
 */
int newServerSocket(const char* port) {
	int serverfd;

	// create new socket file descr		iptor (IPv4, TCP)
	if((serverfd = socket(AF_INET, SOCK_STREAM, 0)) == -1){
		perror("Error creating server socket\n");
		return -1;
	}
	
	// bind socket to port
	struct sockaddr_in serveraddr;
	bzero(&serveraddr, sizeof(serveraddr));
	serveraddr.sin_family = AF_INET;
	serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
	serveraddr.sin_port = htons(atoi(port));

//	printf("serveraddr.sin_port: %d\t %d\n", serveraddr.sin_port, atoi(port));
	if(bind(serverfd, (struct sockaddr*)&serveraddr, sizeof(serveraddr)) == -1){
	 	perror("Error binding server socket\n");
		return -1;
	}

	// listen for incoming connecitons
	if(listen(serverfd, BACKLOG) == -1){
		perror("Error in server listen\n");
		return -1;
	}

//	printf("listening at %d\n", serverfd);
	
	return serverfd;
}

/**
 * creates a new socket file descriptor, connects to host and port
 */

int newClientSocket(const char *host, const char *port){
	int clientfd;

	// create client socket file descriptor
	if((clientfd = socket(AF_INET, SOCK_STREAM, 0)) == -1){
		perror("Error creaitng client socket");
		return -1;
	}
	
	// connect to server
	struct hostent* he;
	struct sockaddr_in hostaddr;

	if((he = gethostbyname(host)) == NULL){
		perror("Error in gethostbyname\n");
		return -1;
	}

	hostaddr.sin_family = AF_INET;
	hostaddr.sin_port = htons(atoi(port));
	hostaddr.sin_addr = *((struct in_addr*)he->h_addr);
	bzero(&(hostaddr.sin_zero), 8);

	if(connect(clientfd, (struct sockaddr*)&hostaddr, sizeof(hostaddr)) == -1){
		perror("Error connecting to host");
		return -1;
	}
	
	return clientfd;
}

/**
 * accepts new connections on a server socket and returns the socket
 * file descriptor for the client.
 */
int acceptSocket(int serverfd) {
	int acceptfd;
	 
//	printf("Accepting new connection at %i.\n", serverfd);

//	struct sockaddr_storage clientaddr;
//	bzero(&clientaddr, sizeof(clientaddr));

//	unsigned addrlen = sizeof(clientaddr);

	while((acceptfd = accept(serverfd, NULL, NULL)) == -1){
		printf("Error accepting connecion at %i\n:", serverfd);
	}
//	printf("Accepted new connection at %d\n", acceptfd);	
	
	return acceptfd;
}

/**
 * closes socket file descriptor.
 */
void closeSocket(int sockfd) {

	if (sockfd <= 0){
		perror("Invalid file descriptor passed ot closeSocket()\n");
	}

	if(close(sockfd) == -1)
		printf("Error closing sockfd.\n");
}



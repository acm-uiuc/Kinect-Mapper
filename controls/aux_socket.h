#ifndef __AUX_SOCKET_H__
#define __AUX_SOCKET_H__

/**
 * creates a new socket file descriptor, binds to the local port port,
 * then sets up a listening queue.
 */
int newServerSocket(const char* port);

/**
 * creates a new socket file descriptor, connects to host and port
 */

int newClientSocket(const char* host, const char *port);

/**
 * accepts new connections on a server socket and returns the socket
 * file descriptor for the client.
 */
int acceptSocket(int serverFd);

/**
 * closes socket file descriptor.
 */
void closeSocket(int sockFd);

#endif

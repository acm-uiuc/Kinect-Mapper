
#ifndef SERVER_H
#define SERVER_H

#include <unistd.h>
#include <string>
using std::string;

class Server{
 public:
  Server();
  ~Server();
  int setupServer();
  int acceptConnection(int serverfd);
  char getMessage(int acceptfd);
  
 protected:
  char* port;
  string host;
  int serverfd;
  int acceptfd;
};

#endif

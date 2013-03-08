
#ifndef SERVER_H
#define SERVER_H

#include <termios.h>
#include <unistd.h>
#include <string>
using std::string;

class Server{
 public:
  Server();
  ~Server();
  int setupServer();
  int acceptConnection();
  char getMessage();
  
 protected:
  char* port;
  string host;
  int serverfd;
  int acceptfd;

  struct termios settings;
};

#endif


#ifndef SERVER_H
#define SERVER_H

#include "robot.h"
#include <termios.h>
#include <unistd.h>
#include <string>
using std::string;

class Server{
 public:
  Server(Robot* rob);
  Server(Robot* rob, char* pt); 
  int setupServer();
  int acceptConnection();
  char getMessage();
  
 protected:
  void sigINT_handler(int s);
  static void* arduino_data_reader(void* args);

  char* port;
  string host;
  int serverfd;
  int acceptfd;

  struct termios settings;

  Robot* bob;
};

#endif

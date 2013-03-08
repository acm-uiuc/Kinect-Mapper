
#ifndef INTERFACE_H
#define INTERFACE_H

#include "server.h"
#include "robot.h"
#include <termios.h>

#define MODE_MANUAL 0
#define MODE_PLANNER 1

#define STOP 0
#define FORWARD 1
#define NODATA -1


class Interface{
 public:
  Interface();
  ~Interface();
  
  int run();
  void setMode(int mode);
  void passCommand(int command, int source);
  int sendMessageToRobot(char message, int mode);

 protected:
  struct termios settings;
  int createAndDetach();
  int writeToSerial();
  void* runServer(void* args);
  int setupArduinoConnection();
  void* arduino_data_reader(void* args);
  Server* getServer();

  Robot* rob;
  Server* server;
//  MapperPathPlanner* planner;
  char* port;

  int ser;

  char* buf;
  int BUFSIZE;
  int CNUM;
  speed_t baud;
};

#endif


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
#define LEFT 2
#define RIGHT -2


class Interface{
 public:
  Interface();
  ~Interface();
  
  int run();
  void setMode(int mode);
  void passCommand(int command, int source);
  int sendMessageToRobot(char message, int mode);
  static int afd;
  static int sfd;
  int setupArduinoConnection();

 protected:
  struct termios settings;
  int createAndDetach();
  int writeToSerial();
  void* runServer(void* args);
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
  
  pthread_attr_t server_attr;
  pthread_t server_thread_id;

  pthread_attr_t attr;
  pthread_t thread_id;
  
};

#endif

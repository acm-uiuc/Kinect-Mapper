#include "StateInformation.h"
#include "kinect_interface.h"
#include "odometry.h"
#include "graph.h"
#include "planning.h"
#include "interface.h"
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>

using std::cout;
using std::endl;

static void* serverThread(void* args){
    Interface* interface = (Interface*)args;
	interface->run();
	pthread_exit(NULL);
}

static void* arduinoThread(void* args){
	Interface* interface = (Interface*)args;
	interface->setupArduinoConnection();
	pthread_exit(NULL);
}


int main(int argc, char** argv)
{
  //TODO: This is only a basic idea of what we will need to do for this
  // section.  We will need to update calls and the control loop as other
  // sections get filled out.
  
  // counter here for now, later on we will have to remove it
  int totalSteps = 100;
  int currStep = 1;
  int height = 480, width = 640;
  fovis::CameraIntrinsicsParameters rgb_params;
  memset(&rgb_params, 0, sizeof(fovis::CameraIntrinsicsParameters));
  rgb_params.width = width;
  rgb_params.height = height;
  rgb_params.fx = 528.49404721; 
  rgb_params.fy = rgb_params.fx;
  rgb_params.cx = width / 2.0;
  rgb_params.cy = height / 2.0;
  fovis::Rectification rect(rgb_params);
  
  VisGraph map;
  KinectInter camera(rgb_params);
  if (!camera.initialize()) {
    cout << "Camera Initialization Failed." << endl;
    return 1;
  }
  if (!camera.startDataCapture()) {
    cout << "Unable to start capturing data" << endl;
    return 1;
  }
  
  RGBDVisOdometry odom(rgb_params);
  MapperPathPlanner planner(width,height);

  Interface* interface = new Interface();

  pthread_t ard_id;
  pthread_attr_t ard_attr;
  if(pthread_attr_init(&ard_attr) != 0)
    printf("Error setting thread attributes.\n");
  if(pthread_create(&ard_id, &ard_attr, arduinoThread, interface) != 0)
    printf("Error arduino_data_reader thread.\n");
  while(pthread_detach(ard_id) != 0)
    printf("Error detaching thread\n");

  pthread_t thread_id;
  pthread_attr_t attr;
  if(pthread_attr_init(&attr) != 0)
    printf("Error setting thread attributes.\n");
  if(pthread_create(&thread_id, &attr, serverThread, interface) != 0)
    printf("Error arduino_data_reader thread.\n");
  while(pthread_detach(thread_id) != 0)
    printf("Error detaching thread\n");

  while (currStep < totalSteps) {
    // Get RGBD data from current frame
    if (camera.captureOne()) {
      FrameDataPtr currFrame = camera.getFrame();
      
      // Get Transformation from previous frame
      //odom.getMotionEstimate(currFrame);
      if (planner.canMove(currFrame))
	cout << "Can Move" << endl;
      else
	cout << "Can't move" << endl;
      // Add new node to the graph
      //map.addNode(currFrame);
      // Get next movement command
      char cmd = planner.getNextCommand(currFrame);
      // Execute command
      //interface.passCommand(cmd, MODE_PLANNER);
      // Wait for command to finish
    }
    int waitlength = 2;
    sleep(waitlength);
    // Prepare for next iteration
    currStep++;
  }
  camera.stopDataCapture();
  pthread_exit(NULL);
  return 0;
}

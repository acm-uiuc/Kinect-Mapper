#include "StateInformation.h"
#include "kinect_interface.h"
#include "odometry.h"
#include "graph.h"
#include "planning.h"
#include "interface.h"
#include <unistd.h>
using std::cout;
using std::endl;

int main(int argc, char** argv)
{
  //TODO: This is only a basic idea of what we will need to do for this
  // section.  We will need to update calls and the control loop as other
  // sections get filled out.
  
  // counter here for now, later on we will have to remove it
  int totalSteps = 100;
  int currStep = 1;
  
  VisGraph map;
  KinectInter camera;
  if (!camera.initialize()) {
    cout << "Camera Initialization Failed." << endl;
    return 1;
  }
  if (!camera.startDataCapture()) {
    cout << "Unable to start capturing data" << endl;
    return 1;
  }
  //RGBDVisOdometry odom(camera);
  MapperPathPlanner planner;
  Interface interface;
  interface.run();

  while (currStep < totalSteps) {
    // Get RGBD data from current frame
    if (camera.captureOne()) {
      FrameDataPtr currFrame = camera.getFrame();
      // Get Transformation from previous frame
      // odom.getMotionEstimate(currFrame);
      if (planner.canMove(currFrame))
	cout << "Can Move" << endl;
      else
	cout << "Can't move" << endl;
      // Add new node to the graph
      //map.addNode(currFrame);
      // Get next movement command
      //int cmd = planner.getNextCommand(currFrame);
      // Execute command
      // interface.passCommand(cmd, MODE_PLANNER);
      // Wait for command to finish
    }
    int waitlength = 5;
    sleep(waitlength);
    // Prepare for next iteration
    currStep++;
  }
  camera.stopDataCapture();
  return 0;
}

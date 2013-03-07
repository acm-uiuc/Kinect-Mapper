#include "StateInformation.h"
#include "kinect_interface.h"
#include "odometry.h"
#include "graph.h"
#include "planning.h"
#include <unistd.h>

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
  RGBDVisOdometry odom(camera);
  MapperPathPlanner planner;

  while (currStep < totalSteps) {
    // Get RGBD data from current frame
    FrameDataPtr currFrame = camera.getFrame();
    // Get Transformation from previous frame
    odom.getMotionEstimate(currFrame);
    // Add new node to the graph
    map.addNode(currFrame);
    // Get next movement command
    int cmd = planner.getNextCommand(currFrame);
    // Execute command
    // TODO: add call to arduino to execute command
    // Wait for command to finish
    int waitlength = 5;
    sleep(waitlength);
    // Prepare for next iteration
    currStep++;
  }

  return 0;
}

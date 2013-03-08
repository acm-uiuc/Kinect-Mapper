/************************************************************************
*************************************************************************
 Reminder: None of the functions are set in stone, these are just in place
 to provide guideance on what we will need to implement.  Make whatever 
 changes you think are necessary.
 ************************************************************************
 ***********************************************************************/

#ifndef PLANNING_H
#define PLANNING_H

#include "StateInformation.h"
#include "kinect_interface.h"

class MapperPathPlanner
{
 public:
  MapperPathPlanner();
  ~MapperPathPlanner();
  char getNextCommand(FrameDataPtr);
  bool canMove(FrameDataPtr);
 protected:
  //enum Command{STOP, NOCHANGE, FORWARD, BACKWARD, RTURN, LTURN, SUP, SDOWN};
  float minDepth;    
  int depthSumThreshold;
  int width_;
  int height_;

  int NODATA;
  int STOP;
  int FORWARD;
};

#endif

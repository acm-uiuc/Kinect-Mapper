/************************************************************************
*************************************************************************
 Reminder: None of the functions are set in stone, these are just in place
 to provide guideance on what we will need to implement.  Make whatever 
 changes you think are necessary.
 ************************************************************************
 ***********************************************************************/

#ifndef NAVIGATION_H
#define NAVIGATION_H

struct OdomTrans 
{
  // Translation
  double x,y,z;
  // Euler Angles
  double alpha,beta,gamma;
};

struct FrameData
{
  OdomTrans trans; 
  
};

#include "graph/graph.h"
#include "odometry/odometry.h"
#include "planning/planning.h"
#include "utilities/kinect_interface.h"

#endif

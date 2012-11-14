/************************************************************************
*************************************************************************
 Reminder: None of the functions are set in stone, these are just in place
 to provide guideance on what we will need to implement.  Make whatever 
 changes you think are necessary.
 ************************************************************************
 ***********************************************************************/

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "navigation.h"
#include <fovis/fovis.hpp>

class RGBDVisOdometry
{
 public:
  RGBDVisOdometry();
  ~RGBDVisOdometry();
  void getTransformation(FrameData* prevFrame, FrameData* currFrame);
 protected:
  fovis::VisualOdometry*  Odom_;
};

#endif

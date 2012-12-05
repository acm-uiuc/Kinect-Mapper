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
#include "../utilities/kinect_interface.h"
#include <fovis/fovis.hpp>

class RGBDVisOdometry
{
 public:
  RGBDVisOdometry(const KinectInter& camera);
  ~RGBDVisOdometry();
  OdomTrans& getMotionEstimate(FrameData* currFrame = NULL);
  OdomTrans& getPose(FrameData* currFrame = NULL);
  OdomTrans& Isometry3DToOdomTrans(Eigen::Isometry3d odomInfo);
 protected:
  fovis::VisualOdometry*  Odom_;
};

#endif

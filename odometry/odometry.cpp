/************************************************************************
*************************************************************************
 Reminder: None of the functions are set in stone, these are just in place
 to provide guideance on what we will need to implement.  Make whatever 
 changes you think are necessary.
 ************************************************************************
 ***********************************************************************/

#include "odometry.h"

RGBDVisOdometry::RGBDVisOdometry(const KinectInter& camera)
{
  //TODO: Set up visual odometry options
  /*fovis::VisualOdometryOptions options = 
      fovis::VisualOdometry::getDefaultOptions();
  fovis::Rectification rect(camera.getCameraParameters());
  Odom_ = new fovis::VisualOdometry(&rect, options);*/
}

RGBDVisOdometry::~RGBDVisOdometry()
{
  delete Odom_;
}

/*************************************************************
 *
 *
 ***********************************************************/
OdomTrans& RGBDVisOdometry::getMotionEstimate(FrameData* currFrame /* = NULL */)
{/*
  // Process frame if a new frame is added
  if (currFrame != NULL)
    Odom_->processFrame(currFrame->grayImage, currFrame->depthImage);

  // Extract pose
  Eigen::Isometry3d pose = Odom_->getMotionEstimate();

  // Fill in struct and return requested information
  return Isometry3DToOdomTrans(pose);*/
  OdomTrans trans;
  return trans;
}

/*************************************************************
 *
 *
 ***********************************************************/
OdomTrans& RGBDVisOdometry::getPose(FrameData* currFrame /* = NULL */)
{/*
  // Process frame if a new frame is added
  if (currFrame != NULL)
    Odom_->processFrame(currFrame->grayImage, currFrame->depthImage);

  // Extract pose
  Eigen::Isometry3d pose = Odom_->getPose();

  // Fill in struct and return requested information
  return Isometry3DToOdomTrans(pose);*/
  OdomTrans trans;
  return trans;
}

/*************************************************************
 *
 *
 ***********************************************************/
OdomTrans& Isometry3DToOdomTrans(Eigen::Isometry3d info)
{
  /*
  Eigen::Vector3d xyz = m.translation();
  Eigen::Vector3d rpy = m.rotation().eulerAngles(0, 1, 2);*/

  OdomTrans odomInfo;/*
  odomInfo.x = xyz(0);
  odomInfo.y = xyz(1);
  odomInfo.z = syz(2);
  odomInfo.alpha = rpy(0);
  odomInfo.beta = rpy(1);
  odomInfo.gamma = rpy(2);*/

  return odomInfo;
}

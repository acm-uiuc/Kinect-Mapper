/************************************************************************
*************************************************************************
 Reminder: None of the functions are set in stone, these are just in place
 to provide guideance on what we will need to implement.  Make whatever 
 changes you think are necessary.
 ************************************************************************
 ***********************************************************************/

#include "odometry.h"

RGBDVisOdometry::RGBDVisOdometry(const fovis::Rectification& rect)
  : Odom_(new fovis::VisualOdometry(&rect,
				    fovis::VisualOdometry::getDefaultOptions()))
{
  //TODO: Set up visual odometry options
}

RGBDVisOdometry::~RGBDVisOdometry()
{
}

/*************************************************************
 *
 *
 ***********************************************************/
OdomTransPtr RGBDVisOdometry::getMotionEstimate(FrameDataPtr currFrame)
{
  // Process frame if a new frame is added
  if (currFrame)
    Odom_->processFrame(currFrame->gray_image, currFrame->depth_image);

  // Extract pose
  Eigen::Isometry3d pose = Odom_->getMotionEstimate();

  // Fill in struct and return requested information
  return Isometry3DToOdomTrans(pose);
}

/*************************************************************
 *
 *
 ***********************************************************/
OdomTransPtr RGBDVisOdometry::getPose(FrameDataPtr currFrame)
{
  // Process frame if a new frame is added
  if (currFrame != NULL)
    Odom_->processFrame(currFrame->gray_image, currFrame->depth_image);

  // Extract pose
  Eigen::Isometry3d pose = Odom_->getPose();

  // Fill in struct and return requested information
  return Isometry3DToOdomTrans(pose);
  //OdomTrans trans;
  //return trans;
}

/*************************************************************
 *
 *
 ***********************************************************/
OdomTransPtr RGBDVisOdometry::Isometry3DToOdomTrans(Eigen::Isometry3d info)
{
  Eigen::Vector3d xyz = info.translation();
  Eigen::Vector3d rpy = info.rotation().eulerAngles(0, 1, 2);

  OdomTransPtr odomInfo(new OdomTrans);
  odomInfo->x = xyz(0);
  odomInfo->y = xyz(1);
  odomInfo->z = xyz(2);
  odomInfo->alpha = rpy(0);
  odomInfo->beta = rpy(1);
  odomInfo->gamma = rpy(2);
  return odomInfo;
}

/************************************************************************
*************************************************************************
 Reminder: None of the functions are set in stone, these are just in place
 to provide guideance on what we will need to implement.  Make whatever 
 changes you think are necessary.
 ************************************************************************
 ***********************************************************************/

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "StateInformation.h"
#include "kinect_interface.h"
#include "fovis.hpp"

/** \brief
 * This class interfaces with the fovis library to perform visual odometry
 * with the reference frame based on the first frame.
 */
class RGBDVisOdometry
{
 public:

  /** \brief
   * Constructor, initializes the default values for 
   * \param[in] camera input camera parameters for kinect interface
   */
  RGBDVisOdometry(const KinectInter& camera);

  /** \brief
   * Deconstructor, frees any allocated memory 
   */
  ~RGBDVisOdometry();

  /** \brief
   * Gets the current motion estimate from the previous frame to the current frame
   * \param[in] currFrame the frame of the position of the robot to base the estimate from
   * \return the transformation from the previous frame to the current one
   */
  OdomTrans& getMotionEstimate(FrameData* currFrame = NULL);

  /** \brief
   * Gets the current pose estimate of the robot
   * \param[in] currFrame the frame of the position of the robot to base the estimate from
   * \return the transformation containing the pose data
   */
  OdomTrans& getPose(FrameData* currFrame = NULL);

  /** \brief
   * Converts Eigen Isometry3d type to the OdomTrans type
   * \param[in] odomInfo contains the Isometry3d type to convert
   * \return the transformation containing Isometry3d information
   */
  OdomTrans& Isometry3DToOdomTrans(Eigen::Isometry3d odomInfo);
 protected:
  fovis::VisualOdometry*  Odom_;
};

#endif


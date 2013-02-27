/************************************************************************
*************************************************************************
 Reminder: None of the functions are set in stone, these are just in place
 to provide guideance on what we will need to implement.  Make whatever 
 changes you think are necessary.
 ************************************************************************
 ***********************************************************************/

#ifndef STATEINFORMATION_H
#define STATEINFORMATION_H

#include <boost/shared_ptr.hpp>

/** \brief
 * Stores odometry information about the robot
 */
struct OdomTrans 
{
  /** \brief
   * Stores translations where +z is forward, +x is right, and +y is down
   */
  double x,y,z;

  /** \brief
   * Stores the Euler Angles that encodes the robot's pose
   */
  double alpha,beta,gamma;
};

/** \brief
 * Stores the data about the current state of the robot
 */
struct FrameData
{
  OdomTrans trans; 
  
};

typedef boost::shared_ptr<FrameData> FrameDataPtr;
typedef boost::shared_ptr<OdomTrans> OdomTransPtr;

#endif
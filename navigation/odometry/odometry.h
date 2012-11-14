/************************************************************************
*************************************************************************
 Reminder: None of the functions are set in stone, these are just in place
 to provide guideance on what we will need to implement.  Make whatever 
 changes you think are necessary.
 ************************************************************************
 ***********************************************************************/

#ifndef ODOMETRY_H
#def ODOMETRY_H

#include <fovis/fovis.hpp>

stuct OdomTrans 
{
  // Translation
  double x,y,z;
  // Euler Angles
  double alpha,beta,gamma;
}

class RGBDVisOdometry
{
 public:
  RGBDVisOdometry();
  ~RGBDVisOdometry();
  void getTransformation(OdomTrans& trans);
 protected:
  fovis::VisualOdometry  Odom_;
}

#endif

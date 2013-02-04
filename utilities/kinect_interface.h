/************************************************************************
*************************************************************************
 Reminder: None of the functions are set in stone, these are just in place
 to provide guideance on what we will need to implement.  Make whatever 
 changes you think are necessary.
 ************************************************************************
 ***********************************************************************/

#ifndef KINECT_INTERFACE_H
#define KINECT_INTERFACE_H

#include "navigation.h"

class KinectInter
{
 public:
  KinectInter();
  ~KinectInter();
  FrameData* getFrame();
 protected:
  //Add any members needed to access the kinect
};

#endif

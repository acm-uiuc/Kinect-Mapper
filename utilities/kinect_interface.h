/************************************************************************
*************************************************************************
* Reminder: None of the functions are set in stone, these are just in place
* to provide guideance on what we will need to implement.  Make whatever 
* changes you think are necessary.
*
* This class is adapted from libfovis data_capture examples
*************************************************************************
************************************************************************/

#ifndef KINECT_INTERFACE_H
#define KINECT_INTERFACE_H

#include "StateInformation.h"
#include "fovis.hpp"
#include <XnOS.h>
#include <XnCppWrapper.h>
#include <vector>
using std::vector;

class KinectInter
{
 public:
  KinectInter();
  ~KinectInter();
  bool initialize();
  bool captureOne();
  FrameDataPtr getFrame();
  bool startDataCapture();
  bool stopDataCapture();
 protected:

  xn::Context context_;
  xn::EnumerationErrors errors_;

  xn::DepthGenerator depth_gen_;
  xn::DepthMetaData depth_md_;
  xn::ImageGenerator image_gen_;
  xn::ImageMetaData image_md_;

  boost::shared_ptr<fovis::DepthImage> depth_image_;

  int width_;
  int height_;

  fovis::CameraIntrinsicsParameters rgb_params_;

  float* depth_data_;

  vector<uint8_t> gray_buf_;
};

#endif

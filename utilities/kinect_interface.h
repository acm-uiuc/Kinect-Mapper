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
#include <libfreenect/libfreenect.h>
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
  static void depth_cb(freenect_device *dev, void *data, uint32_t timestamp);
  static void image_cb(freenect_device *dev, void *data, uint32_t timestamp);

  void DepthCallback(void* data, uint32_t timestamp);
  void ImageCallback(void* data, uint32_t timestamp);
  freenect_context *f_ctx_;
  freenect_device *f_dev_;

  int freenect_angle_;
  int device_number_;

  fovis::DepthImage* depth_image_;

  int width_;
  int height_;
  bool have_image_;
  bool have_depth_;

  fovis::CameraIntrinsicsParameters rgb_params_;

  float* depth_data_;
  uint8_t* gray_buf_;
};

#endif

/************************************************************************
*************************************************************************
 Reminder: None of the functions are set in stone, these are just in place
 to provide guideance on what we will need to implement.  Make whatever 
 changes you think are necessary.
 ************************************************************************
 ***********************************************************************/

#include "kinect_interface.h"
#include <cstdio>

#define CHECK_STATUS(rc, msg) if((rc) != XN_STATUS_OK) { \
  fprintf(stderr, "%s: %s\n", (msg), xnGetStatusString(rc)); return false; }

KinectInter::KinectInter()
{
  width_ = 640;
  height_ = 480;

  memset(&rgb_params_, 0, sizeof(fovis::CameraIntrinsicsParameters));
  rgb_params_.width = width_;
  rgb_params_.height = height_;

  // TODO read these values from the camera somehow, instead of hard-coding it
  // Unfortunately, the OpenNI API doesn't seem to expose them.
  rgb_params_.fx = 528.49404721; 
  rgb_params_.fy = rgb_params_.fx;
  rgb_params_.cx = width_ / 2.0;
  rgb_params_.cy = height_ / 2.0;

  depth_image_ = new fovis::DepthImage(rgb_params_, width_, 
						     height_);

  depth_data_ = new float[width_ * height_];
  gray_buf_ = new uint8_t[width_ * height_];

  freenect_angle_ = 0;
  device_number_ = 0;
}

KinectInter::~KinectInter()
{
  delete[] depth_data_;
  delete [] gray_buf_;
}

FrameDataPtr KinectInter::getFrame()
{
  FrameDataPtr framePtr(new FrameData);
  framePtr->depth_image = depth_image_;
  memcpy(gray_buf_,framePtr->gray_image,width_ * height_ * sizeof(uint8_t));
  return framePtr;
}


bool KinectInter::initialize()
{
  // initialize the kinect device
  if (freenect_init(&f_ctx_, NULL) < 0) {
    printf("freenect_init() failed\n");
    return false;
  }

  freenect_set_log_level(f_ctx_, FREENECT_LOG_ERROR);

  int num_devices = freenect_num_devices(f_ctx_);
  printf("Number of devices found: %d\n", num_devices);

  if (num_devices < 1)
    return false;

  if (freenect_open_device(f_ctx_, &f_dev_, device_number_) < 0) {
    printf("Could not open device\n");
    return false;
  }

  freenect_set_user(f_dev_, this);

  freenect_frame_mode vmode = freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB);
  freenect_frame_mode dmode = freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_REGISTERED);

  freenect_set_video_mode(f_dev_, vmode);
  freenect_set_depth_mode(f_dev_, dmode);

  return true;
}

bool KinectInter::startDataCapture()
{
  // start data capture
  printf("Starting data capture\n");
  freenect_set_tilt_degs(f_dev_, freenect_angle_);
  freenect_set_led(f_dev_, LED_OFF);
  freenect_set_depth_callback(f_dev_, &KinectInter::depth_cb);
  freenect_set_video_callback(f_dev_, &KinectInter::image_cb);

  freenect_start_depth(f_dev_);
  freenect_start_video(f_dev_);
  return true;
}

bool KinectInter::stopDataCapture()
{
  freenect_stop_depth(f_dev_);
  freenect_stop_video(f_dev_);

  freenect_close_device(f_dev_);
  freenect_shutdown(f_ctx_);
  return true;
}

bool KinectInter::captureOne()
{
 while (freenect_process_events(f_ctx_) >= 0) {
    if (have_image_ && have_depth_) {
      have_image_ = false;
      have_depth_ = false;
      return true;
    }
  }
  return false;
}


void KinectInter::depth_cb(freenect_device *dev, void *data, uint32_t timestamp)
{
  KinectInter* self = (KinectInter*)(freenect_get_user(dev));
  self->DepthCallback(data, timestamp);
}

void KinectInter::image_cb(freenect_device *dev, void *data, uint32_t timestamp)
{
  KinectInter* self = (KinectInter*)(freenect_get_user(dev));
  self->ImageCallback(data, timestamp);
}

void KinectInter::DepthCallback(void* data, uint32_t timestamp)
{
  have_depth_ = true;

  uint16_t* depth_mm = (uint16_t*)data;
  int num_pixels = width_ * height_;
  for(int i=0; i<num_pixels; i++) {
    uint16_t d = depth_mm[i];
    if(d != 0) {
      depth_data_[i] = d * 1e-3;
    } else {
      depth_data_[i] = NAN;
    }
  }
  depth_image_->setDepthImage(depth_data_);
}

void KinectInter::ImageCallback(void* data, uint32_t timestamp)
{
  have_image_ = true;

  int num_pixels = width_ * height_;
  uint8_t* rgb_pixel = (uint8_t*)data;
  uint8_t* gray_pixel = gray_buf_;
  for(int i=0; i<num_pixels; i++) {
    gray_pixel[0] = (rgb_pixel[0] + rgb_pixel[1] + rgb_pixel[2]) / 3;
    gray_pixel++;
    rgb_pixel += 3;
  }
}

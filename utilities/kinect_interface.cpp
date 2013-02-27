/************************************************************************
*************************************************************************
 Reminder: None of the functions are set in stone, these are just in place
 to provide guideance on what we will need to implement.  Make whatever 
 changes you think are necessary.
 ************************************************************************
 ***********************************************************************/

#include "kinect_interface.h"

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

  fovis::DepthImage* depthIm = new fovis::DepthImage(rgb_params_, width_, 
						     height_);
  depth_image_ = boost::shared_ptr<fovis::DepthImage>(depthIm);

  depth_data_ = new float[width_ * height_];
  gray_buf_.resize(width_ * height_,0);
}

KinectInter::~KinectInter()
{
  delete[] depth_data_;
}

FrameDataPtr KinectInter::getFrame()
{
  FrameDataPtr framePtr(new FrameData);
  framePtr->depth_image = depth_image_;
  framePtr->gray_image = gray_buf_;
  return framePtr;
}


bool KinectInter::initialize()
{
  XnStatus rc = context_.Init();
  CHECK_STATUS(rc, "Initializing device context");

  printf("Initializing image stream\n");
  image_gen_.Create(context_);
  rc = image_gen_.Create(context_);
  CHECK_STATUS(rc, "Initializing image stream");

  // set output format to RGB
  image_gen_.SetPixelFormat(XN_PIXEL_FORMAT_RGB24);

  XnMapOutputMode image_mode;
  image_mode.nXRes = width_;
  image_mode.nYRes = height_;
  image_mode.nFPS = 30;
  image_gen_.SetMapOutputMode(image_mode);
  CHECK_STATUS(rc, "Setting image output mode");


  printf("Initializing depth stream\n");
  rc = depth_gen_.Create(context_);
  CHECK_STATUS(rc, "Initializing depth stream");

  depth_gen_.SetMapOutputMode(image_mode);
  CHECK_STATUS(rc, "Setting depth output mode");

  depth_gen_.GetMetaData(depth_md_);
  printf("Depth offset: %d %d\n", depth_md_.XOffset(), depth_md_.YOffset());
  // XXX do we need to do something with the depth offset?

  // set the depth image viewpoint
  depth_gen_.GetAlternativeViewPointCap().SetViewPoint(image_gen_);

  // read off the depth camera field of view.  This is the FOV corresponding to
  // the IR camera viewpoint, regardless of the alternative viewpoint settings.
  XnFieldOfView fov;
  rc = depth_gen_.GetFieldOfView(fov);
  return true;
}

bool KinectInter::startDataCapture()
{
  // start data capture
  printf("Starting data capture\n");
  XnStatus rc = context_.StartGeneratingAll();
  CHECK_STATUS(rc, "Starting data capture");
  return true;
}

bool KinectInter::stopDataCapture()
{
  context_.StopGeneratingAll();
  context_.Release();
  return true;
}

bool KinectInter::captureOne()
{
  // Read a new frame
  XnStatus rc = context_.WaitAndUpdateAll();
  CHECK_STATUS(rc, "Reading frame");

  // grab the image data
  image_gen_.GetMetaData(image_md_);
  const XnRGB24Pixel* rgb_data = image_md_.RGB24Data();

  // convert to grayscale.
  int num_rgb_pixels = width_ * height_;
  for(int i=0; i<num_rgb_pixels; i++) {
//    gray_buf[i] = (int)round((rgb_data->nRed + 
//                              rgb_data->nGreen + 
//                              rgb_data->nBlue) / 3.0);
    gray_buf_[i] = (int)round(0.2125 * rgb_data->nRed + 
                             0.7154 * rgb_data->nGreen + 
                             0.0721 * rgb_data->nBlue);
    rgb_data++;
  }

  // grab the depth data
  depth_gen_.GetMetaData(depth_md_);
  int depth_data_nbytes = width_ * height_ * sizeof(uint16_t);
  const uint16_t* depth_data_u16 = depth_md_.Data();

  // convert to meters, and set unknown depth values to NAN
  int num_depth_pixels = width_ * height_;
  for(int i=0; i<num_depth_pixels; i++) {
    uint16_t d = depth_data_u16[i];
    if(d != 0) {
      depth_data_[i] = d * 1e-3;
    } else {
      depth_data_[i] = NAN;
    }
  }

  depth_image_->setDepthImage(depth_data_);
  return true;
}

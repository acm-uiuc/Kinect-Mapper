/* This file is part of RGBDSLAM.
 * 
 * RGBDSLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * RGBDSLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with RGBDSLAM.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Core>
#include <QString>
#include <QMatrix4x4>
#include <ctime>
#include <limits>
#include "parameter_server.h"
#include <cv.h>

#include <g2o/math_groups/se3quat.h>

#include <pcl_ros/transforms.h>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"

void printQMatrix4x4(const char* name, const QMatrix4x4& m){
    ROS_DEBUG("QMatrix %s:", name);
    ROS_DEBUG("%f\t%f\t%f\t%f", m(0,0), m(0,1), m(0,2), m(0,3));
    ROS_DEBUG("%f\t%f\t%f\t%f", m(1,0), m(1,1), m(1,2), m(1,3));
    ROS_DEBUG("%f\t%f\t%f\t%f", m(2,0), m(2,1), m(2,2), m(2,3));
    ROS_DEBUG("%f\t%f\t%f\t%f", m(3,0), m(3,1), m(3,2), m(3,3));
}

void printTransform(const char* name, const tf::Transform t) {
    ROS_INFO_STREAM(name << ": Translation " << t.getOrigin().x() << " " << t.getOrigin().y() << " " << t.getOrigin().z());
    ROS_INFO_STREAM(name << ": Rotation " << t.getRotation().getX() << " " << t.getRotation().getY() << " " << t.getRotation().getZ() << " " << t.getRotation().getW());
}

void logTransform(QTextStream& out, const tf::Transform& t, double timestamp, const char* label) {
    if(label) out << label << ": ";
    out << timestamp << " " << t.getOrigin().x() << " " << t.getOrigin().y() << " " << t.getOrigin().z() << " " << t.getRotation().getX() << " " << t.getRotation().getY() << " " << t.getRotation().getZ() << " " << t.getRotation().getW() << "\n";
}


QMatrix4x4 g2o2QMatrix(const g2o::SE3Quat se3) {
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
    Eigen::Matrix<double, 4, 4> m = se3.to_homogenious_matrix(); //_Matrix< 4, 4, double >
    ROS_DEBUG_STREAM("Eigen Matrix:\n" << m);
    QMatrix4x4 qmat( static_cast<qreal*>( m.data() )  );
    // g2o/Eigen seems to use a different row-major/column-major array layout
    printQMatrix4x4("from conversion", qmat.transposed());//thus the transposes
    return qmat.transposed();//thus the transposes
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}

tf::Transform g2o2TF(const g2o::SE3Quat se3) {
    tf::Transform result;
    tf::Vector3 translation;
    translation.setX(se3.translation().x());
    translation.setY(se3.translation().y());
    translation.setZ(se3.translation().z());

    tf::Quaternion rotation;
    rotation.setX(se3.rotation().x());
    rotation.setY(se3.rotation().y());
    rotation.setZ(se3.rotation().z());
    rotation.setW(se3.rotation().w());

    result.setOrigin(translation);
    result.setRotation(rotation);
    //printTransform("from conversion", result);
    return result;
}
//From: /opt/ros/unstable/stacks/perception_pcl/pcl/src/pcl/registration/transforms.hpp
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Apply an affine transform defined by an Eigen Transform
  * \param cloud_in the input point cloud
  * \param cloud_to_append_to the transformed cloud will be appended to this one
  * \param transform a tf::Transform stating the transformation of cloud_to_append_to relative to cloud_in
  * \note The density of the point cloud is lost, since density implies that the origin is the point of view
  * \note Can not(?) be used with cloud_in equal to cloud_to_append_to
  */
//template <typename PointT> void
//transformAndAppendPointCloud (const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_to_append_to,
//                              const tf::Transform transformation)
void transformAndAppendPointCloud (const pointcloud_type &cloud_in, 
                                   pointcloud_type &cloud_to_append_to,
                                   const tf::Transform transformation, float max_Depth)
{
    bool compact = !ParameterServer::instance()->get<bool>("preserve_raster_on_save");
    Eigen::Matrix4f eigen_transform;
    pcl_ros::transformAsMatrix(transformation, eigen_transform);
    unsigned int cloud_to_append_to_original_size = cloud_to_append_to.size();
    if(cloud_to_append_to.points.size() ==0){
        cloud_to_append_to.header   = cloud_in.header;
        cloud_to_append_to.width    = 0;
        cloud_to_append_to.height   = 0;
        cloud_to_append_to.is_dense = false;
    }

    ROS_DEBUG("max_Depth = %f", max_Depth);
    ROS_DEBUG("cloud_to_append_to_original_size = %i", cloud_to_append_to_original_size);

    //Append all points untransformed
    cloud_to_append_to += cloud_in;

    Eigen::Matrix3f rot   = eigen_transform.block<3, 3> (0, 0);
    Eigen::Vector3f trans = eigen_transform.block<3, 1> (0, 3);
    point_type origin = point_type();
    origin.x = 0;
    origin.y = 0;
    origin.z = 0;
    int j = 0;
    for (size_t i = 0; i < cloud_in.points.size (); ++i)
    { 
      Eigen::Map<Eigen::Vector3f> p_in (const_cast<float*>(&cloud_in.points[i].x), 3, 1);
      Eigen::Map<Eigen::Vector3f> p_out (&cloud_to_append_to.points[j+cloud_to_append_to_original_size].x, 3, 1);
      if(compact){ cloud_to_append_to.points[j+cloud_to_append_to_original_size] = cloud_in.points[i]; }
      //filter out points with a range greater than the given Parameter or do nothing if negativ
      if(max_Depth >= 0){
        if(pcl::squaredEuclideanDistance(cloud_in.points[i], origin) > max_Depth*max_Depth){
           p_out[0]= std::numeric_limits<float>::quiet_NaN();
           p_out[1]= std::numeric_limits<float>::quiet_NaN();
           p_out[2]= std::numeric_limits<float>::quiet_NaN();
           if(!compact) j++; 
           continue;
         }
      }
      if (pcl_isnan (cloud_in.points[i].x) || pcl_isnan (cloud_in.points[i].y) || pcl_isnan (cloud_in.points[i].z)){
         if(!compact) j++;
         continue;
      }
      p_out = rot * p_in + trans;
      j++;
    }
    if(compact){
      cloud_to_append_to.points.resize(j+cloud_to_append_to_original_size);
      cloud_to_append_to.width    = 1;
      cloud_to_append_to.height   = j+cloud_to_append_to_original_size;
	}
}

//do spurious type conversions
geometry_msgs::Point pointInWorldFrame(const Eigen::Vector4f& point3d, g2o::SE3Quat transf){
    Eigen::Vector3d tmp(point3d[0], point3d[1], point3d[2]);
    tmp = transf * tmp; //transform to world frame
    geometry_msgs::Point p;
    p.x = tmp.x(); 
    p.y = tmp.y(); 
    p.z = tmp.z();
    return p;
}
void mat2dist(const Eigen::Matrix4f& t, double &dist){
    dist = sqrt(t(0,3)*t(0,3)+t(1,3)*t(1,3)+t(2,3)*t(2,3));
}
///Get euler angles from affine matrix (helper for isBigTrafo)
void mat2RPY(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw) {
    roll = atan2(t(2,1),t(2,2));
    pitch = atan2(-t(2,0),sqrt(t(2,1)*t(2,1)+t(2,2)*t(2,2)));
    yaw = atan2(t(1,0),t(0,0));
}
void mat2components(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw, double& dist){

  mat2RPY(t, roll,pitch,yaw);
  mat2dist(t, dist);

  roll = roll/M_PI*180;
  pitch = pitch/M_PI*180;
  yaw = yaw/M_PI*180;

}

// true iff edge qualifies for generating a new vertex
bool isBigTrafo(const Eigen::Matrix4f& t){
    double roll, pitch, yaw, dist;

    mat2RPY(t, roll,pitch,yaw);
    mat2dist(t, dist);

    roll = roll/M_PI*180;
    pitch = pitch/M_PI*180;
    yaw = yaw/M_PI*180;

    double max_angle = std::max(roll,std::max(pitch,yaw));

    // at least 10cm or 5deg
    return (dist > ParameterServer::instance()->get<double>("min_translation_meter")
    		|| max_angle > ParameterServer::instance()->get<int>("min_rotation_degree"));
}

bool isBigTrafo(const g2o::SE3Quat& t){
    float angle_around_axis = 2.0*acos(t.rotation().w()) *180.0 / M_PI;
    float dist = t.translation().norm();
    QString infostring;
    ROS_INFO("Rotation:% 4.2f, Distance: % 4.3fm", angle_around_axis, dist);
    infostring.sprintf("Rotation:% 4.2f, Distance: % 4.3fm", angle_around_axis, dist);
    //Q_EMIT setGUIInfo2(infostring);
    ParameterServer* ps =  ParameterServer::instance();
    if(dist > ps->get<double>("min_translation_meter") ||
    	 angle_around_axis > ps->get<int>("min_rotation_degree"))
    {
      //Too big fails too
      if(dist < ps->get<double>("max_translation_meter") ||
    		 angle_around_axis < ps->get<int>("max_rotation_degree"))
      {
        return true;
      }
      else
      {
        ROS_INFO("Rejected Transformation because it is too large");
      }
    }
    return false;
}


/*
bool overlappingViews(LoadedEdge3D edge){
    //opening angles
   double alpha = 57.0/180.0*M_PI;
   double beta = 47.0/180.0*M_PI; 
   //assumes robot coordinate system (x is front, y is left, z is up)
   Eigen::Matrix<double, 4,3> cam1;
   cam1 <<  1.0, std::tan(alpha),  std::tan(beta),//upper left
                                       1.0, -std::tan(alpha), std::tan(beta),//upper right
                                       1.0, std::tan(alpha), -std::tan(beta),//lower left
                                       1.0, -std::tan(alpha),-std::tan(beta);//lower right
   return false;
}
bool triangleRayIntersection(Eigen::Vector3d triangle1,Eigen::Vector3d triangle2, 
                             Eigen::Vector3d ray_origin, Eigen::Vector3d ray){
    Eigen::Matrix3d m;
    m.col(2) = -ray;
    m.col(0) = triangle1;
    m.col(1) = triangle2;
    Eigen::Vector3d lengths = m.inverse() * ray_origin;
    return (lengths(0) < 0 && lengths(1) > 0 && lengths(2) > 0 );
}
*/


g2o::SE3Quat tf2G2O(const tf::Transform t) 
{
  Eigen::Quaterniond eigen_quat(t.getRotation().getW(), t.getRotation().getX(), t.getRotation().getY(), t.getRotation().getZ());
  Eigen::Vector3d translation(t.getOrigin().x(), t.getOrigin().y(), t.getOrigin().z());
  g2o::SE3Quat result(eigen_quat, translation);
  return result;
}

g2o::SE3Quat eigen2G2O(const Eigen::Matrix4d& eigen_mat) 
{
  Eigen::Affine3d eigen_transform(eigen_mat);
  Eigen::Quaterniond eigen_quat(eigen_transform.rotation());
  Eigen::Vector3d translation(eigen_mat(0, 3), eigen_mat(1, 3), eigen_mat(2, 3));
  g2o::SE3Quat result(eigen_quat, translation);

  return result;
}

using namespace cv;
///Analog to opencv example file and modified to use adjusters
FeatureDetector* createDetector( const string& detectorType ) 
{
	ParameterServer* params = ParameterServer::instance();
	FeatureDetector* fd = 0;
    if( !detectorType.compare( "FAST" ) ) {
        //fd = new FastFeatureDetector( 20/*threshold*/, true/*nonmax_suppression*/ );
        fd = new DynamicAdaptedFeatureDetector (new FastAdjuster(20,true), 
												params->get<int>("min_keypoints"),
												params->get<int>("max_keypoints"),
												params->get<int>("adjuster_max_iterations"));
    }
    else if( !detectorType.compare( "STAR" ) ) {
        fd = new StarFeatureDetector( 16/*max_size*/, 5/*response_threshold*/, 10/*line_threshold_projected*/,
                                      8/*line_threshold_binarized*/, 5/*suppress_nonmax_size*/ );
    }
    else if( !detectorType.compare( "SIFT" ) ) {
        fd = new SiftFeatureDetector();//SIFT::DetectorParams::GET_DEFAULT_THRESHOLD(),
                                     //SIFT::DetectorParams::GET_DEFAULT_EDGE_THRESHOLD());
        ROS_INFO("Default SIFT threshold:"); /* %f, Default SIFT Edge Threshold: %f", 
                 SIFT::DetectorParams::GET_DEFAULT_THRESHOLD(),
                 SIFT::DetectorParams::GET_DEFAULT_EDGE_THRESHOLD());*/
    }
    else if( !detectorType.compare( "SURF" ) ) {
      /* fd = new SurfFeatureDetector(200.0, 6, 5); */
        fd = new DynamicAdaptedFeatureDetector(new SurfAdjuster(),
        										params->get<int>("min_keypoints"),
                            params->get<int>("max_keypoints")+300,
                            params->get<int>("adjuster_max_iterations"));
    }
    else if( !detectorType.compare( "MSER" ) ) {
        fd = new MserFeatureDetector( 1/*delta*/, 60/*min_area*/, 114400/*_max_area*/, 0.35f/*max_variation*/,
                0.2/*min_diversity*/, 200/*max_evolution*/, 1.01/*area_threshold*/, 0.003/*min_margin*/,
                5/*edge_blur_size*/ );
    }
    else if( !detectorType.compare( "GFTT" ) ) {
        fd = new GoodFeaturesToTrackDetector( 200/*maxCorners*/, 0.001/*qualityLevel*/, 1./*minDistance*/,
                                              5/*int _blockSize*/, true/*useHarrisDetector*/, 0.04/*k*/ );
    }
#if CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION >= 3
    else if( !detectorType.compare( "ORB" ) ) {
        fd = new OrbFeatureDetector();//params->get<int>("max_keypoints")+1500,
                //ORB::CommonParams(1.2, ORB::CommonParams::DEFAULT_N_LEVELS, 31, ORB::CommonParams::DEFAULT_FIRST_LEVEL));
    }
#endif
    else if( !detectorType.compare( "SIFTGPU" ) ) {
      ROS_INFO("%s is to be used", detectorType.c_str());
      ROS_DEBUG("Creating SURF detector as fallback.");
      fd = createDetector("SURF"); //recursive call with correct parameter
    }
    else {
      ROS_WARN("No valid detector-type given: %s. Using SURF.", detectorType.c_str());
      fd = createDetector("SURF"); //recursive call with correct parameter
    }
    ROS_ERROR_COND(fd == 0, "No detector could be created");
    return fd;
}

DescriptorExtractor* createDescriptorExtractor( const string& descriptorType ) 
{
    DescriptorExtractor* extractor = 0;
    if( !descriptorType.compare( "SIFT" ) ) {
        extractor = new SiftDescriptorExtractor();/*( double magnification=SIFT::DescriptorParams::GET_DEFAULT_MAGNIFICATION(), bool isNormalize=true, bool recalculateAngles=true, int nOctaves=SIFT::CommonParams::DEFAULT_NOCTAVES, int nOctaveLayers=SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS, int firstOctave=SIFT::CommonParams::DEFAULT_FIRST_OCTAVE, int angleMode=SIFT::CommonParams::FIRST_ANGLE )*/
    }
    else if( !descriptorType.compare( "SURF" ) ) {
        extractor = new SurfDescriptorExtractor();/*( int nOctaves=4, int nOctaveLayers=2, bool extended=false )*/
    }
#if CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION >= 3
    else if( !descriptorType.compare( "ORB" ) ) {
        extractor = new OrbDescriptorExtractor();
    }
#endif
    else if( !descriptorType.compare( "SIFTGPU" ) ) {
      ROS_INFO("%s is to be used as extractor, creating SURF descriptor extractor as fallback.", descriptorType.c_str());
      extractor = new SurfDescriptorExtractor();/*( int nOctaves=4, int nOctaveLayers=2, bool extended=false )*/
    }
    else {
      ROS_ERROR("No valid descriptor-matcher-type given: %s. Using SURF", descriptorType.c_str());
      extractor = createDescriptorExtractor("SURF");
    }
    ROS_ERROR_COND(extractor == 0, "No extractor could be created");
    return extractor;
}

void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img){
  //Process images
  if(mono8_img.rows != float_img.rows || mono8_img.cols != float_img.cols){
    mono8_img = cv::Mat(float_img.size(), CV_8UC1);}
  cv::convertScaleAbs(float_img, mono8_img, 100, 0.0);
  //The following doesn't work due to NaNs
  //double minVal, maxVal; 
  //minMaxLoc(float_img, &minVal, &maxVal);
  //ROS_DEBUG("Minimum/Maximum Depth in current image: %f/%f", minVal, maxVal);
  //mono8_img = cv::Scalar(0);
  //cv::line( mono8_img, cv::Point2i(10,10),cv::Point2i(200,100), cv::Scalar(255), 3, 8);
}

//Little debugging helper functions
std::string openCVCode2String(unsigned int code){
  switch(code){
    case 0 : return std::string("CV_8UC1" );
    case 8 : return std::string("CV_8UC2" );
    case 16: return std::string("CV_8UC3" );
    case 24: return std::string("CV_8UC4" );
    case 2 : return std::string("CV_16UC1");
    case 10: return std::string("CV_16UC2");
    case 18: return std::string("CV_16UC3");
    case 26: return std::string("CV_16UC4");
    case 5 : return std::string("CV_32FC1");
    case 13: return std::string("CV_32FC2");
    case 21: return std::string("CV_32FC3");
    case 29: return std::string("CV_32FC4");
  }
  return std::string("Unknown");
}

void printMatrixInfo(cv::Mat& image, std::string name){
  ROS_INFO_STREAM("Matrix " << name << " - Type:" << openCVCode2String(image.type()) <<  " rows: " <<  image.rows  <<  " cols: " <<  image.cols);
}

bool asyncFrameDrop(ros::Time depth, ros::Time rgb)
{
  long rgb_timediff = abs(static_cast<long>(rgb.nsec) - static_cast<long>(depth.nsec));
  if(rgb_timediff > 33333333){
     ROS_INFO("Depth image time: %d - %d", depth.sec,   depth.nsec);
     ROS_INFO("RGB   image time: %d - %d", rgb.sec, rgb.nsec);
     ROS_WARN("Depth and RGB image off more than 1/30sec: %li (nsec)", rgb_timediff);
     if(ParameterServer::instance()->get<bool>("drop_async_frames")){
       ROS_WARN("Asynchronous frames ignored. See parameters if you want to keep async frames.");
       return true;
     }
  } else {
     ROS_DEBUG("Depth image time: %d - %d", depth.sec,   depth.nsec);
     ROS_DEBUG("RGB   image time: %d - %d", rgb.sec, rgb.nsec);
  }
  return false;
}


#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

///\cond
/** Union for easy "conversion" of rgba data */
typedef union
{
  struct /*anonymous*/
  {
    unsigned char Blue;
    unsigned char Green;
    unsigned char Red;
    unsigned char Alpha;
  };
  float float_value;
  long long_value;
} RGBValue;
///\endcond

pointcloud_type* createXYZRGBPointCloud (const sensor_msgs::ImageConstPtr& depth_msg, 
                                         const sensor_msgs::ImageConstPtr& rgb_msg1,
                                         const sensor_msgs::CameraInfoConstPtr& cam_info) 
{
  sensor_msgs::ImagePtr rgb_msg(new sensor_msgs::Image);
  rgb_msg->header = rgb_msg1->header;
  rgb_msg->height = rgb_msg1->height;
  rgb_msg->width = rgb_msg1->width;
  rgb_msg->encoding = rgb_msg1->encoding;
  rgb_msg->is_bigendian = rgb_msg1->is_bigendian;
  rgb_msg->step = rgb_msg1->step;
  rgb_msg->data = rgb_msg1->data;

  if (rgb_msg->encoding.compare("bgr8") == 0) {
    rgb_msg->encoding = "rgb8";
    for (int i=0; i<rgb_msg->data.size(); i+=3) {
      char temp_red = rgb_msg->data[i];
      rgb_msg->data[i] = rgb_msg->data[i+2];
      rgb_msg->data[i+2] = temp_red;
    }
  }
  struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
  pointcloud_type* cloud (new pointcloud_type() );
  cloud->header.stamp     = depth_msg->header.stamp;
  cloud->header.frame_id  = rgb_msg->header.frame_id;
  cloud->is_dense         = true; //single point of view, 2d rasterized

  float cx, cy, fx, fy;//principal point and focal lengths
  unsigned color_step, color_skip;

  cloud->height = depth_msg->height;
  cloud->width = depth_msg->width;
  cx = cam_info->K[2]; //(cloud->width >> 1) - 0.5f;
  cy = cam_info->K[5]; //(cloud->height >> 1) - 0.5f;
  fx = 1.0f / cam_info->K[0]; 
  fy = 1.0f / cam_info->K[4]; 
  int pixel_data_size = 3;
  char red_idx = 0, green_idx = 1, blue_idx = 2;
  if(rgb_msg->encoding.compare("mono8") == 0) pixel_data_size = 1;
  if(rgb_msg->encoding.compare("bgr8") == 0) { red_idx = 2; blue_idx = 0; }


  ROS_ERROR_COND(pixel_data_size == 0, "Unknown image encoding: %s!", rgb_msg->encoding.c_str());
  color_step = pixel_data_size * rgb_msg->width / cloud->width;
  color_skip = pixel_data_size * (rgb_msg->height / cloud->height - 1) * rgb_msg->width;

  cloud->points.resize (cloud->height * cloud->width);

  const float* depth_buffer = reinterpret_cast<const float*>(&depth_msg->data[0]);
  const uint8_t* rgb_buffer = &rgb_msg->data[0];

  // depth_msg already has the desired dimensions, but rgb_msg may be higher res.
  int color_idx = 0, depth_idx = 0;
  double depth_scaling = ParameterServer::instance()->get<double>("depth_scaling_factor");

  pointcloud_type::iterator pt_iter = cloud->begin ();
  for (int v = 0; v < (int)cloud->height; ++v, color_idx += color_skip)
  {
    for (int u = 0; u < (int)cloud->width; ++u, color_idx += color_step, ++depth_idx, ++pt_iter)
    {
      point_type& pt = *pt_iter;
      float Z = depth_buffer[depth_idx] * depth_scaling;

      // Check for invalid measurements
      if (std::isnan (Z))
      {
        pt.x = pt.y = pt.z = Z;
      }
      else // Fill in XYZ
      {
        pt.x = (u - cx) * Z * fx;
        pt.y = (v - cy) * Z * fy;
        pt.z = Z;
      }

      // Fill in color
      RGBValue color;
      if(pixel_data_size == 3){
        color.Red   = rgb_buffer[color_idx + red_idx];
        color.Green = rgb_buffer[color_idx + green_idx];
        color.Blue  = rgb_buffer[color_idx + blue_idx];
      } else {
        color.Red   = color.Green = color.Blue  = rgb_buffer[color_idx];
      }
      color.Alpha = 0;
      pt.rgb = color.float_value;
    }
  }

  clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
  return cloud;
}

double errorFunction(const Eigen::Vector4f& x1, const double x1_depth_cov, 
                     const Eigen::Vector4f& x2, const double x2_depth_cov, 
                     const Eigen::Matrix4f& tf_1_to_2)
{
  const double cam_angle_x = 58.0/180.0*M_PI;
  const double cam_angle_y = 45.0/180.0*M_PI;
  const double cam_resol_x = 640;
  const double cam_resol_y = 480;
  const double raster_stddev_x = 2*tan(cam_angle_x/cam_resol_x);  //2pix stddev in x
  const double raster_stddev_y = 2*tan(cam_angle_y/cam_resol_y);  //2pix stddev in y
  const double raster_cov_x = raster_stddev_x * raster_stddev_x;
  const double raster_cov_y = raster_stddev_y * raster_stddev_y;

  ROS_WARN_COND(x1(3) != 1.0, "4th element of x1 should be 1.0, is %f", x1(3));
  ROS_WARN_COND(x2(3) != 1.0, "4th element of x2 should be 1.0, is %f", x2(3));
  
  Eigen::Vector3d mu_1 = x1.head<3>().cast<double>();
  Eigen::Vector3d mu_2 = x2.head<3>().cast<double>();
  Eigen::Matrix3d rotation_mat = tf_1_to_2.block(0,0,3,3).cast<double>();

  //Point 1
  Eigen::Matrix3d cov1 = Eigen::Matrix3d::Identity();
  cov1(0,0) = raster_cov_x* mu_1(2); //how big is 1px std dev in meter, depends on depth
  cov1(1,1) = raster_cov_y* mu_1(2); //how big is 1px std dev in meter, depends on depth
  cov1(2,2) = x1_depth_cov;
  //Point2
  Eigen::Matrix3d cov2 = Eigen::Matrix3d::Identity();
  cov2(0,0) = raster_cov_x* mu_2(2); //how big is 1px std dev in meter, depends on depth
  cov2(1,1) = raster_cov_y* mu_2(2); //how big is 1px std dev in meter, depends on depth
  cov2(2,2) = x2_depth_cov;

  Eigen::Matrix3d cov2inv = cov2.inverse(); // Σ₂⁻¹  

  Eigen::Vector3d mu_1_in_frame_2 = (tf_1_to_2 * x1).head<3>().cast<double>(); // μ₁⁽²⁾  = T₁₂ μ₁⁽¹⁾  
  Eigen::Matrix3d cov1_in_frame_2 = rotation_mat.transpose() * cov1 * rotation_mat;//Works since the cov is diagonal => Eig-Vec-Matrix is Identity
  Eigen::Matrix3d cov1inv_in_frame_2 = cov1_in_frame_2.inverse();// Σ₁⁻¹  

  Eigen::Matrix3d cov_sum = (cov1inv_in_frame_2 + cov2inv);
  Eigen::Matrix3d inv_cov_sum = cov_sum.inverse();
  ROS_ERROR_STREAM_COND(inv_cov_sum!=inv_cov_sum,"Sum of Covariances not invertible: \n" << cov_sum);

  Eigen::Vector3d x_ml;//Max Likelhood Position of latent point, that caused the sensor msrmnt
  x_ml = inv_cov_sum * (cov1inv_in_frame_2 * mu_1_in_frame_2 + cov2inv * mu_2); // (Σ₁⁻¹ +  Σ₂⁻¹)⁻¹(Σ₁⁻¹μ₁  +  Σ₂⁻¹μ₂)
  Eigen::Vector3d delta_mu_1 = mu_1_in_frame_2 - x_ml;
  Eigen::Vector3d delta_mu_2 = mu_2 - x_ml;
  
  float sqrd_mahalanobis_distance1 = delta_mu_1.transpose() * cov1inv_in_frame_2 * delta_mu_1;// Δx_2^T Σ Δx_2 
  float sqrd_mahalanobis_distance2 = delta_mu_2.transpose() * cov2inv * delta_mu_2; // Δx_1^T Σ Δx_1
  float bad_mahalanobis_distance = sqrd_mahalanobis_distance1 + sqrd_mahalanobis_distance2; //FIXME

  if(!(bad_mahalanobis_distance >= 0.0))
  {
    ROS_ERROR_STREAM("Non-Positive Mahalanobis Distance");
    return std::numeric_limits<double>::max();
  }
  ROS_DEBUG_STREAM_NAMED("statistics", "Mahalanobis ML: " << std::setprecision(25) << bad_mahalanobis_distance);
  return bad_mahalanobis_distance;
}


double errorFunction2(const Eigen::Vector4f& x1,
                      const Eigen::Vector4f& x2,
                      const Eigen::Matrix4f& tf_1_to_2)
{
  static const double cam_angle_x = 58.0/180.0*M_PI;/*{{{*/
  static const double cam_angle_y = 45.0/180.0*M_PI;
  static const double cam_resol_x = 640;
  static const double cam_resol_y = 480;
  static const double raster_stddev_x = 2*tan(cam_angle_x/cam_resol_x);  //2pix stddev in x
  static const double raster_stddev_y = 2*tan(cam_angle_y/cam_resol_y);  //2pix stddev in y
  static const double raster_cov_x = raster_stddev_x * raster_stddev_x;
  static const double raster_cov_y = raster_stddev_y * raster_stddev_y;/*}}}*/

  ROS_WARN_COND(x1(3) != 1.0, "4th element of x1 should be 1.0, is %f", x1(3));
  ROS_WARN_COND(x2(3) != 1.0, "4th element of x2 should be 1.0, is %f", x2(3));

  Eigen::Vector3d mu_1 = x1.head<3>().cast<double>();
  Eigen::Vector3d mu_1_in_frame_2 = (tf_1_to_2 * x1).head<3>().cast<double>(); // μ₁⁽²⁾  = T₁₂ μ₁⁽¹⁾  
  Eigen::Vector3d mu_2 = x2.head<3>().cast<double>();
  Eigen::Matrix3d rotation_mat = tf_1_to_2.block(0,0,3,3).cast<double>();

  //Point 1
  Eigen::Matrix3d cov1 = Eigen::Matrix3d::Zero();
  cov1(0,0) = 1 * raster_cov_x * mu_1(2); //how big is 1px std dev in meter, depends on depth
  cov1(1,1) = 1 * raster_cov_y * mu_1(2); //how big is 1px std dev in meter, depends on depth
  cov1(2,2) = mu_1(2)*mu_1(2) * 0.0075; //stddev computed from information on http://www.ros.org/wiki/openni_kinect/kinect_accuracy
  //Point2
  Eigen::Matrix3d cov2 = Eigen::Matrix3d::Zero();
  cov2(0,0) = 1 * raster_cov_x* mu_2(2); //how big is 1px std dev in meter, depends on depth
  cov2(1,1) = 1 * raster_cov_y* mu_2(2); //how big is 1px std dev in meter, depends on depth
  cov1(2,2) = mu_2(2)*mu_2(2) * 0.0075; //stddev computed from information on http://www.ros.org/wiki/openni_kinect/kinect_accuracy

  Eigen::Matrix3d cov1_in_frame_2 = rotation_mat.transpose() * cov1 * rotation_mat;//Works since the cov is diagonal => Eig-Vec-Matrix is Identity

  // Δμ⁽²⁾ =  μ₁⁽²⁾ - μ₂⁽²⁾
  Eigen::Vector3d delta_mu_in_frame_2 = mu_1_in_frame_2 - mu_2;
  // Σc = (Σ₁ + Σ₂)
  Eigen::Matrix3d cov_mat_sum_in_frame_2 = cov1_in_frame_2 + cov2;     
  //ΔμT Σc⁻¹Δμ  
  double sqrd_mahalanobis_distance = delta_mu_in_frame_2.transpose() * cov_mat_sum_in_frame_2.inverse() * delta_mu_in_frame_2;
  
  if(!(sqrd_mahalanobis_distance >= 0.0))
  {
    ROS_ERROR_STREAM("Non-Positive Mahalanobis Distance for vectors " << x1 << "\nand\n" << x2 << " with covariances\n" << cov1 << "\nand\n" << cov2);
    return std::numeric_limits<double>::max();
  }
  //ROS_INFO_STREAM_NAMED("statistics", "Probability: " << std::setprecision(10) << probability << " Normalization: " << normalization);
  ROS_DEBUG_STREAM_NAMED("statistics", "Mahalanobis Distance: " << sqrd_mahalanobis_distance);
  ROS_DEBUG_STREAM_NAMED("statistics", "Covariance Matrix:\n" << cov_mat_sum_in_frame_2);
  ROS_DEBUG_STREAM_NAMED("statistics", "Sigma 1:\n" << cov1 << "\nSigma 2:\n" << cov2);
  ROS_DEBUG_STREAM_NAMED("statistics", "Sigma 1 in Frame 2:\n" << cov1_in_frame_2 << "\nDelta mu: " << delta_mu_in_frame_2);
  ROS_DEBUG_STREAM_NAMED("statistics", "Transformation Matrix:\n" << tf_1_to_2 << "\nRotation:\n" << rotation_mat.transpose() * rotation_mat);
  return sqrd_mahalanobis_distance;
}

float getMinDepthInNeighborhood(const cv::Mat& depth, cv::Point2f center, float diameter){
    // Get neighbourhood area of keypoint
    int radius = (diameter - 1)/2;
    int top   = center.y - radius; top   = top   < 0 ? 0 : top;
    int left  = center.x - radius; left  = left  < 0 ? 0 : left;
    int bot   = center.y + radius; bot   = bot   > depth.rows ? depth.rows : bot;
    int right = center.x + radius; right = right > depth.cols ? depth.cols : right;

    cv::Mat neigborhood(depth, cv::Range(top, bot), cv::Range(left,right));
    double minZ = std::numeric_limits<float>::quiet_NaN();
    cv::minMaxLoc(neigborhood, &minZ);
    if(minZ == 0.0){ //FIXME: Why are there features with depth set to zero?
      ROS_WARN_THROTTLE(1,"Caught feature with zero in depth neighbourhood");
      minZ = std::numeric_limits<float>::quiet_NaN();
    }

    return static_cast<float>(minZ);
}

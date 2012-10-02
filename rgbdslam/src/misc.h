#ifndef RGBD_SLAM_MISC_H_
#define RGBD_SLAM_MISC_H_
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
#include <QMatrix4x4>

void printTransform(const char* name, const tf::Transform t) ;
///Write Transformation to textstream
void logTransform(QTextStream& out, const tf::Transform& t, double timestamp, const char* label = NULL);
void printQMatrix4x4(const char* name, const QMatrix4x4& m);

///Conversion Function
QMatrix4x4    g2o2QMatrix(const g2o::SE3Quat se3) ;
///Conversion Function
tf::Transform g2o2TF(     const g2o::SE3Quat se3) ;
///Conversion Function
g2o::SE3Quat  eigen2G2O(  const Eigen::Matrix4d& eigen_mat);
///Conversion Function
g2o::SE3Quat  tf2G2O(     const tf::Transform t);

/// get euler angles and translation from 4x4 homogenous
void mat2components(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw, double& dist);
/// get euler angles from 4x4 homogenous
void mat2RPY(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw);
/// get translation-distance from 4x4 homogenous
void mat2dist(const Eigen::Matrix4f& t, double &dist);


///Creates a pointcloud from rgb8 or mono8 coded images + float depth
pointcloud_type* createXYZRGBPointCloud (const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::CameraInfoConstPtr& cam_info); 

///Helper function to aggregate pointclouds in a single coordinate frame
void transformAndAppendPointCloud (const pointcloud_type &cloud_in, pointcloud_type &cloud_to_append_to,
                                   const tf::Transform transformation, float Max_Depth);


geometry_msgs::Point pointInWorldFrame(const Eigen::Vector4f& point3d, g2o::SE3Quat transf);
// true if translation > 10cm or largest euler-angle>5 deg
// used to decide if the camera has moved far enough to generate a new nodes
bool isBigTrafo(const Eigen::Matrix4f& t);
bool isBigTrafo(const g2o::SE3Quat& t);


//bool overlappingViews(LoadedEdge3D edge);
//bool triangleRayIntersection(Eigen::Vector3d triangle1,Eigen::Vector3d triangle2, Eigen::Vector3d ray_origin, Eigen::Vector3d ray);


/// Creates Feature Detector Objects accordingt to the type.
/// Possible detectorTypes: FAST, STAR, SIFT, SURF, GFTT
/// FAST and SURF are the self-adjusting versions (see http://opencv.willowgarage.com/documentation/cpp/features2d_common_interfaces_of_feature_detectors.html#DynamicAdaptedFeatureDetector)
cv::FeatureDetector* createDetector( const std::string& detectorType );
/// Create an object to extract features at keypoints. The Exctractor is passed to the Node constructor and must be the same for each node.
cv::DescriptorExtractor* createDescriptorExtractor( const std::string& descriptorType );
///Convert the CV_32FC1 image to CV_8UC1 with a fixed scale factor
void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img);

///Return the macro string for the cv::Mat type integer
std::string openCVCode2String(unsigned int code);

///Print Type and size of image
void printMatrixInfo(cv::Mat& image, std::string name = std::string(""));

//!Return true if frames should be dropped because they are asynchronous
bool asyncFrameDrop(ros::Time depth, ros::Time rgb);

double errorFunction(const Eigen::Vector4f& x1, const double x1_depth_cov, 
                      const Eigen::Vector4f& x2, const double x2_depth_cov, 
                      const Eigen::Matrix4f& tf_1_to_2);

double errorFunction2(const Eigen::Vector4f& x1, 
                      const Eigen::Vector4f& x2, 
                      const Eigen::Matrix4f& tf_1_to_2);

float getMinDepthInNeighborhood(const cv::Mat& depth, cv::Point2f center, float diameter);

#endif

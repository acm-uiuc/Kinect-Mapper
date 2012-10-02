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


#include "node.h"
#include <cmath>
#include <ctime>
#include <Eigen/Geometry>
#include "pcl/ros/conversions.h"
#include <pcl/common/transformation_from_correspondences.h>
//#include <opencv2/highgui/highgui.hpp>
#include <qtconcurrentrun.h>
#include <QtConcurrentMap> 

#ifdef USE_SIFT_GPU
#include "sift_gpu_wrapper.h"
#endif

//#include <math.h>
#include <fstream>
#ifdef USE_ICP_BIN
#include "gicp-fallback.h"
#endif

#ifdef USE_ICP_CODE
#include "../external/gicp/transform.h"
#endif

//#include <iostream>
#include "misc.h"
#include <pcl/filters/voxel_grid.h>
#include <opencv/highgui.h>

QMutex Node::gicp_mutex;
QMutex Node::siftgpu_mutex;

Node::Node(const cv::Mat& visual, 
           const cv::Mat& depth,
           const cv::Mat& detection_mask,
           const sensor_msgs::CameraInfoConstPtr& cam_info, 
           std_msgs::Header depth_header,
           cv::Ptr<cv::FeatureDetector> detector,
           cv::Ptr<cv::DescriptorExtractor> extractor) :
  id_(0), 
  pc_col(new pointcloud_type()),
  flannIndex(NULL),
  base2points_(tf::Transform::getIdentity(), depth_header.stamp, ParameterServer::instance()->get<std::string>("base_frame_name"), depth_header.frame_id),
  ground_truth_transform_(tf::Transform::getIdentity(), depth_header.stamp, ParameterServer::instance()->get<std::string>("ground_truth_frame_name"), ParameterServer::instance()->get<std::string>("base_frame_name")),
  odom_transform_(tf::Transform::getIdentity(), depth_header.stamp, "missing_odometry", depth_header.frame_id),
  initial_node_matches_(0)
{
  ParameterServer* ps = ParameterServer::instance();
  pc_col->header = depth_header;
  struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);

#ifdef USE_SIFT_GPU
  std::vector<float> descriptors;
  if(ps->get<std::string>("feature_detector_type") == "SIFTGPU"){
    SiftGPUWrapper* siftgpu = SiftGPUWrapper::getInstance();
    siftgpu->detect(visual, feature_locations_2d_, descriptors);
    ROS_FATAL_COND(descriptors.size()==0, "Can't run SiftGPU");
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", "Feature Detection and Descriptor Extraction runtime: "<< elapsed <<" s");
  } else 
#endif
  {
    ROS_FATAL_COND(detector.empty(), "No valid detector!");
    detector->detect( visual, feature_locations_2d_, detection_mask);// fill 2d locations
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", "Feature Detection runtime: "<< elapsed <<" s");
  }

  // project pixels to 3dPositions and create search structures for the gicp
#ifdef USE_SIFT_GPU
  if(ps->get<std::string>("feature_detector_type") == "SIFTGPU"){
    projectTo3DSiftGPU(feature_locations_2d_, feature_locations_3d_, depth, cam_info, descriptors, feature_descriptors_); 
  }
  else
#endif
  {
    projectTo3D(feature_locations_2d_, feature_locations_3d_, depth, cam_info);
    struct timespec starttime2; clock_gettime(CLOCK_MONOTONIC, &starttime2);
    extractor->compute(visual, feature_locations_2d_, feature_descriptors_); //fill feature_descriptors_ with information 
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime2.tv_sec); elapsed += (finish.tv_nsec - starttime2.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", "Feature Extraction runtime: "<< elapsed <<" s");
  }
  assert(feature_locations_2d_.size() == feature_locations_3d_.size());
  assert(feature_locations_3d_.size() == (unsigned int)feature_descriptors_.rows); 
  ROS_INFO_NAMED("statistics", "Feature Count of Node:\t%d", (int)feature_locations_2d_.size());
  size_t max_keyp = ps->get<int>("max_keypoints");
  if(feature_locations_2d_.size() > max_keyp) {
    feature_locations_2d_.resize(max_keyp);
    feature_locations_3d_.resize(max_keyp);
    feature_descriptors_ = feature_descriptors_.rowRange(0,max_keyp);
  }
  //computeKeypointDepthStats(depth, feature_locations_2d_);

#ifdef USE_ICP_CODE
  if(ps->get<bool>("use_icp")){
    ROS_ERROR("ICP cannot be used without PointCloud. Wrong Node Constructor");
  }
  gicp_initialized = false;
  gicp_point_set_ = NULL;
#endif
  clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");

}











Node::Node(const cv::Mat visual,
           cv::Ptr<cv::FeatureDetector> detector,
           cv::Ptr<cv::DescriptorExtractor> extractor,
           pointcloud_type::Ptr point_cloud,
           const cv::Mat detection_mask) : 
  id_(0),
  pc_col(point_cloud),
  flannIndex(NULL),
  base2points_(tf::Transform::getIdentity(), point_cloud->header.stamp,ParameterServer::instance()->get<std::string>("base_frame_name"), point_cloud->header.frame_id),
  ground_truth_transform_(tf::Transform::getIdentity(), point_cloud->header.stamp, ParameterServer::instance()->get<std::string>("ground_truth_frame_name"), ParameterServer::instance()->get<std::string>("base_frame_name")),
  odom_transform_(tf::Transform::getIdentity(), point_cloud->header.stamp, "missing_odometry", point_cloud->header.frame_id),
  initial_node_matches_(0)
{
  //cv::namedWindow("matches");
  ParameterServer* ps = ParameterServer::instance();

  ROS_INFO_STREAM("Construction of Node with " << ps->get<std::string>("feature_detector_type") << " Features");
  struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);

#ifdef USE_SIFT_GPU
  std::vector<float> descriptors;
  if(ps->get<std::string>("feature_detector_type") == "SIFTGPU"){
    SiftGPUWrapper* siftgpu = SiftGPUWrapper::getInstance();
    siftgpu->detect(visual, feature_locations_2d_, descriptors);
    ROS_FATAL_COND(descriptors.size() ==0, "Can't run SiftGPU");
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", "Feature Detection and Descriptor Extraction runtime: "<< elapsed <<" s");
  } else 
#endif
  if(ps->get<std::string>("feature_detector_type") != "GICP")
  {
    ROS_FATAL_COND(detector.empty(), "No valid detector!");
    detector->detect( visual, feature_locations_2d_, detection_mask);// fill 2d locations
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", "Feature Detection runtime: "<< elapsed <<" s");
  }

  // project pixels to 3dPositions and create search structures for the gicp
#ifdef USE_SIFT_GPU
  if(ps->get<std::string>("feature_detector_type") == "SIFTGPU")
  {
    // removes also unused descriptors from the descriptors matrix
    // build descriptor matrix and sets siftgpu_descriptors!
    projectTo3DSiftGPU(feature_locations_2d_, feature_locations_3d_, pc_col, descriptors, feature_descriptors_); //takes less than 0.01 sec
  }
  else 
#endif
  if(ps->get<std::string>("feature_detector_type") != "GICP")
  {
    projectTo3D(feature_locations_2d_, feature_locations_3d_, pc_col); //takes less than 0.01 sec
    // projectTo3d need a dense cloud to use the points.at(px.x,px.y)-Call
    struct timespec starttime2; clock_gettime(CLOCK_MONOTONIC, &starttime2);
    extractor->compute(visual, feature_locations_2d_, feature_descriptors_); //fill feature_descriptors_ with information 
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime2.tv_sec); elapsed += (finish.tv_nsec - starttime2.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", "Feature Extraction runtime: "<< elapsed <<" s");
  }

  if(ps->get<std::string>("feature_detector_type") != "GICP")
  {
    assert(feature_locations_2d_.size() == feature_locations_3d_.size());
    assert(feature_locations_3d_.size() == (unsigned int)feature_descriptors_.rows); 
    ROS_INFO_NAMED("statistics", "Feature Count of Node:\t%d", (int)feature_locations_2d_.size());
    size_t max_keyp = ps->get<int>("max_keypoints");
    if(feature_locations_2d_.size() > max_keyp) {
      feature_locations_2d_.resize(max_keyp);
      feature_locations_3d_.resize(max_keyp);
      feature_descriptors_ = feature_descriptors_.rowRange(0,max_keyp);
    }
    assert(feature_locations_2d_.size() == feature_locations_3d_.size());
    assert(feature_locations_3d_.size() == (unsigned int)feature_descriptors_.rows); 
  }

#ifdef USE_ICP_CODE
  gicp_initialized = false;
  gicp_point_set_ = NULL;
  if(!ps->get<bool>("store_pointclouds") && ps->get<bool>("use_icp")) 
  {//if clearing out point clouds, the icp structure needs to be built before
    gicp_mutex.lock();
    gicp_point_set_ = this->getGICPStructure();
    gicp_mutex.unlock();
  }
#endif

  if((!ps->get<bool>("use_glwidget") ||
      !ps->get<bool>("use_gui")) &&
     !ps->get<bool>("store_pointclouds"))
  {
    ROS_WARN("Clearing out points");
    this->clearPointCloud();
  } else if(ps->get<double>("voxelfilter_size") > 0.0) {
    double vfs = ps->get<double>("voxelfilter_size");
    pcl::VoxelGrid<point_type> sor;
    sor.setLeafSize(vfs,vfs,vfs);
    pointcloud_type::ConstPtr const_cloud_ptr = boost::make_shared<pointcloud_type> (*pc_col);                                                                 
    sor.setInputCloud (const_cloud_ptr);
    sor.filter (*pc_col);
  }
  clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");

}

Node::~Node() {
    delete flannIndex;
}

void Node::setOdomTransform(tf::StampedTransform gt){
    odom_transform_ = gt;
}
void Node::setGroundTruthTransform(tf::StampedTransform gt){
    ground_truth_transform_ = gt;
}
void Node::setBase2PointsTransform(tf::StampedTransform& b2p){
    base2points_ = b2p;
}
tf::StampedTransform Node::getOdomTransform(){
    return odom_transform_;
}
tf::StampedTransform Node::getGroundTruthTransform(){
    return ground_truth_transform_;
}
tf::StampedTransform Node::getBase2PointsTransform(){
    return base2points_;
}

void Node::publish(const char* frame, ros::Time timestamp, ros::Publisher pub){
    sensor_msgs::PointCloud2 cloudMessage;
    pcl::toROSMsg((*pc_col),cloudMessage);
    cloudMessage.header.frame_id = frame;
    cloudMessage.header.stamp = timestamp;
    pub.publish(cloudMessage);
    ROS_INFO("Pointcloud with id %i sent with frame %s", id_, frame);
}

#ifdef USE_ICP_CODE
bool Node::getRelativeTransformationTo_ICP_code(const Node* target_node,
                                                Eigen::Matrix4f& transformation,
                                                const Eigen::Matrix4f& initial_transformation)
{
  struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
  dgc_transform_t initial;
  Eigen2GICP(initial_transformation,initial);

  dgc_transform_t final_trafo;
  dgc_transform_identity(final_trafo);

  gicp_mutex.lock();
	dgc::gicp::GICPPointSet* gicp_point_set = this->getGICPStructure();
  ROS_INFO("this'  (%d) Point Set: %d", this->id_, gicp_point_set->Size());
	dgc::gicp::GICPPointSet* target_gicp_point_set = target_node->getGICPStructure();
  ROS_INFO("others (%d) Point Set: %d", target_node->id_, target_gicp_point_set->Size());
  int iterations = gicp_max_iterations;
  if(gicp_point_set->Size() > Node::gicp_min_point_cnt && 
     target_gicp_point_set->Size() > Node::gicp_min_point_cnt)
  {
   iterations = target_gicp_point_set->AlignScan(gicp_point_set, initial, final_trafo, gicp_d_max);
   GICP2Eigen(final_trafo,transformation);
  } else {
    ROS_WARN("GICP Point Sets not big enough. Skipping ICP");
  }
  gicp_mutex.unlock();


  clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
  return iterations <= gicp_max_iterations;
}

void Node::clearGICPStructure() const
{
    gicp_mutex.lock();
    delete gicp_point_set_; gicp_point_set_ = NULL;
    gicp_mutex.unlock();
}
dgc::gicp::GICPPointSet* Node::getGICPStructure(unsigned int max_count) const
{
  struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
  if(max_count == 0) max_count = ParameterServer::instance()->get<int>("gicp_max_cloud_size");
  //Use Cache
  if(gicp_point_set_ != NULL){
    return gicp_point_set_;
  }
  
  dgc::gicp::GICPPointSet* gicp_point_set = new dgc::gicp::GICPPointSet();

  dgc::gicp::GICPPoint g_p;
  g_p.range = -1;
  for(int k = 0; k < 3; k++) {
    for(int l = 0; l < 3; l++) {
      g_p.C[k][l] = (k == l)?1:0;
    }
  }

  std::vector<dgc::gicp::GICPPoint> non_NaN;
  for (unsigned int i=0; i<(*pc_col).points.size(); i++ ){
    point_type&  p = (*pc_col).points.at(i);
    if (!isnan(p.z)) { // add points to candidate pointset for icp
      g_p.x=p.x;
      g_p.y=p.y;
      g_p.z=p.z;
      non_NaN.push_back(g_p);
    }
  }
  float step = non_NaN.size()/static_cast<float>(max_count);
  step =  step < 1.0 ? 1.0 : step; //only skip, don't use points more than once
  for (float i=0; i<non_NaN.size(); i+=step ){
    gicp_point_set->AppendPoint(non_NaN[static_cast<unsigned int>(i)]);
  }
  ROS_INFO("GICP point set size: %i", gicp_point_set->Size() );
  
  if(gicp_point_set->Size() > Node::gicp_min_point_cnt){
    // build search structure for gicp:
    gicp_point_set->SetDebug(true);
    gicp_point_set->SetGICPEpsilon(gicp_epsilon);
    gicp_point_set->BuildKDTree();
    gicp_point_set->ComputeMatrices();
    gicp_point_set->SetMaxIterationInner(8); // as in test_gicp->cpp
    gicp_point_set->SetMaxIteration(gicp_max_iterations);
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
  }
  else
  {
    ROS_WARN("GICP point set too small, this node will not be algined with GICP!");
  }
  //ROS_INFO_STREAM("time for creating the structure: " << ((std::clock()-starttime_gicp*1.0) / (double)CLOCKS_PER_SEC));
  //ROS_INFO_STREAM("current: " << std::clock() << " " << "start_time: " << starttime_gicp);

  gicp_point_set_ = gicp_point_set;
  clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
  return gicp_point_set;
}
#endif

#ifdef USE_ICP_BIN
bool Node::getRelativeTransformationTo_ICP_bin(const Node* target_node,
    Eigen::Matrix4f& transformation,
    const Eigen::Matrix4f* initial_transformation){
  std::clock_t starttime_icp = std::clock();

  bool converged;

  pointcloud_type::Ptr target_cloud = target_node->pc_col;
  if (initial_transformation != NULL)
  {
    pointcloud_type pc2;
    pcl::transformPointCloud((*pc_col),pc2,*initial_transformation);
    converged = gicpfallback(pc2, *target_cloud, transformation);
  }
  else {
    converged = gicpfallback((*pc_col),*target_cloud, transformation); }

  // Paper
  // clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");

  return converged;
}
#endif

// build search structure for descriptor matching
void Node::buildFlannIndex() {
  if (flannIndex == NULL
      && ParameterServer::instance()->get<std::string> ("matcher_type") == "FLANN" 
      && ParameterServer::instance()->get<std::string> ("feature_detector_type") != "GICP"
      && ParameterServer::instance()->get<std::string> ("feature_extractor_type") != "ORB")
  {
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
    //KDTreeIndexParams When passing an object of this type the index constructed will 
    //consist of a set of randomized kd-trees which will be searched in parallel.
    flannIndex = new cv::flann::Index(feature_descriptors_, cv::flann::KDTreeIndexParams(16));
    ROS_INFO("Built flannIndex (address %p) for Node %i", flannIndex, this->id_);
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
  }
}



//TODO: This function seems to be resistant to parallelization probably due to knnSearch
unsigned int Node::featureMatching(const Node* other, std::vector<cv::DMatch>* matches) const 
{
  struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
  assert(matches->size()==0);
  // number of neighbours found (two, to compare the best matches for distinctness
  const int k = 2;
  //unsigned int one_nearest_neighbour = 0, two_nearest_neighbours = 0;

  // number of neighbors found (has to be two, see l. 57)
  double sum_distances = 0.0;
  ParameterServer* ps = ParameterServer::instance();
  //const int min_kp = ps->get<int> ("min_keypoints");

  //using siftgpu, if available and wanted
  if(ps->get<std::string>("feature_detector_type") == "GICP"){
    return 0;
  }
#ifdef USE_SIFT_GPU
  if (ps->get<std::string> ("matcher_type") == "SIFTGPU") {
    siftgpu_mutex.lock();
    sum_distances = SiftGPUWrapper::getInstance()->match(siftgpu_descriptors, feature_descriptors_.rows, other->siftgpu_descriptors, other->feature_descriptors_.rows, matches);
    siftgpu_mutex.unlock();
  }
  else
#endif
  //using BruteForceMatcher for ORB features
  if (ps->get<std::string> ("matcher_type") == "BRUTEFORCE" || 
      ps->get<std::string> ("feature_extractor_type") == "ORB")
  {
    cv::Ptr<cv::DescriptorMatcher> matcher;
    std::string brute_force_type("BruteForce"); //L2 per default
    if(ps->get<std::string> ("feature_extractor_type") == "ORB"){
      brute_force_type.append("-HammingLUT");
    }
    matcher = cv::DescriptorMatcher::create(brute_force_type);
    std::vector< std::vector<cv::DMatch> > bruteForceMatches;
    matcher->knnMatch(feature_descriptors_, other->feature_descriptors_, bruteForceMatches, k);
    double max_dist_ratio_fac = ps->get<double>("nn_distance_ratio");
    //if ((int)bruteForceMatches.size() < min_kp) max_dist_ratio_fac = 1.0; //if necessary use possibly bad descriptors
    srand((long)std::clock());
    for (unsigned int i = 0; i < bruteForceMatches.size(); i++) {
        cv::DMatch m1 = bruteForceMatches[i][0];
        cv::DMatch m2 = bruteForceMatches[i][1];
        float dist_ratio_fac = m1.distance / m2.distance;
        if (dist_ratio_fac < max_dist_ratio_fac) {//this check seems crucial to matching quality
            sum_distances += m1.distance;
            m1.distance = dist_ratio_fac + (float)rand()/(1000.0*RAND_MAX); //add a small random offset to the distance, since later the dmatches are inserted to a set, which omits duplicates and the duplicates are found via the less-than function, which works on the distance. Therefore we need to avoid equal distances, which happens very often for ORB
            matches->push_back(m1);
        } 

    }
    //matcher->match(feature_descriptors_, other->feature_descriptors_, *matches);
  } 
  else if (ps->get<std::string>("matcher_type") == "FLANN" && 
           ps->get<std::string>("feature_extractor_type") != "ORB")
  {
    if (other->flannIndex == NULL) {
        ROS_FATAL("Node %i in featureMatching: flann Index of Node %i was not initialized", this->id_, other->id_);
        return -1;
    }
    int start_feature = 0;
    int sufficient_matches = ps->get<int>("sufficient_matches");
    int num_segments = feature_descriptors_.rows / (sufficient_matches+100.0); //compute number of segments
    if(sufficient_matches <= 0 || num_segments <= 0){
      num_segments=1;
      sufficient_matches = std::numeric_limits<int>::max();
    }
    int num_features = feature_descriptors_.rows / num_segments;                               //compute features per chunk
    for(int seg = 1; start_feature < feature_descriptors_.rows && seg <= num_segments;  seg++){ //search for matches chunkwise
      // compare
      // http://opencv-cocoa.googlecode.com/svn/trunk/samples/c/find_obj.cpp
      cv::Mat indices(num_features, k, CV_32S);
      cv::Mat dists(num_features, k, CV_32F);
      cv::Mat relevantDescriptors = feature_descriptors_.rowRange(start_feature, start_feature+num_features);

      // get the best two neighbors
      struct timespec flannstarttime, flannfinish; double flannelapsed; clock_gettime(CLOCK_MONOTONIC, &flannstarttime);
      other->flannIndex->knnSearch(relevantDescriptors, indices, dists, k, cv::flann::SearchParams(64));
      clock_gettime(CLOCK_MONOTONIC, &flannfinish); flannelapsed = (flannfinish.tv_sec - flannstarttime.tv_sec); flannelapsed += (flannfinish.tv_nsec - flannstarttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(flannelapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", "Flann knnsearch runtime: "<< flannelapsed <<" s");

      //64: The number of times the tree(s) in the index should be recursively traversed. A higher value for this parameter would give better search precision, but also take more time. If automatic configuration was used when the index was created, the number of checks required to achieve the specified precision was also computed, in which case this parameter is ignored.

      int* indices_ptr = indices.ptr<int> (0);
      float* dists_ptr = dists.ptr<float> (0);

      cv::DMatch match;
      double avg_ratio = 0.0;
      double max_dist_ratio_fac = ps->get<double>("nn_distance_ratio");
      for (int i = 0; i < indices.rows; ++i) {
        float dist_ratio_fac =  static_cast<float>(dists_ptr[2 * i]) / static_cast<float>(dists_ptr[2 * i + 1]);
        avg_ratio += dist_ratio_fac;
        //if (indices.rows < min_kp) dist_ratio_fac = 1.0; //if necessary use possibly bad descriptors
        if (max_dist_ratio_fac > dist_ratio_fac) {
          match.queryIdx = i;
          match.trainIdx = indices_ptr[2 * i];
          match.distance = dist_ratio_fac; //dists_ptr[2 * i];
          sum_distances += match.distance;

          assert(match.trainIdx < other->feature_descriptors_.rows);
          assert(match.queryIdx < feature_descriptors_.rows);
          matches->push_back(match);
        }
      }
      ROS_INFO("Feature Matches between Nodes %3d (%4d features) and %3d (%4d features) in segment %d/%d (features %d to %d of first node):\t%4d. Percentage: %f%%, Avg NN Ratio: %f",
                this->id_, (int)this->feature_locations_2d_.size(), other->id_, (int)other->feature_locations_2d_.size(), seg, num_segments, start_feature, start_feature+num_features, 
                (int)matches->size(), (100.0*matches->size())/((float)start_feature+num_features), avg_ratio / (start_feature+num_features));
      if((int)matches->size() > sufficient_matches){
        ROS_INFO("Enough matches. Skipping remaining segments");
        break;
      }
      if((int)matches->size()*num_segments/(float)seg < 0.5*ps->get<int>("min_matches")){
        ROS_INFO("Predicted not enough feature matches, aborting matching process");
        break;
      }
      start_feature += num_features;
    }//for
  }
  else {
      ROS_FATAL_STREAM("Cannot match features:\nNo valid combination for " <<
                       "matcher_type ("           << ps->get<std::string>("matcher_type") << ") and " <<
                       "feature_extractor_type (" << ps->get<std::string>("feature_extractor_type") << ") chosen.");
  }

  ROS_INFO_NAMED("statistics", "count_matrix(%3d, %3d) =  %4d;",
                 this->id_+1, other->id_+1, (int)matches->size());
  ROS_INFO_NAMED("statistics", "dista_matrix(%3d, %3d) =  %f;",
                 this->id_+1, other->id_+1, sum_distances/ (float)matches->size());
  ROS_DEBUG_NAMED("statistics", "Feature Matches between Nodes %3d (%4d features) and %3d (%4d features):\t%4d",
                  this->id_, (int)this->feature_locations_2d_.size(),
                  other->id_, (int)other->feature_locations_2d_.size(),
                  (int)matches->size());

  //ROS_INFO("matches size: %i, rows: %i", (int) matches->size(), feature_descriptors_.rows);

  clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
  //assert(one_nearest_neighbour+two_nearest_neighbours > 0);
  //return static_cast<float>(one_nearest_neighbour) / static_cast<float>(one_nearest_neighbour+two_nearest_neighbours);
  return matches->size();
}



#ifdef USE_SIFT_GPU
void Node::projectTo3DSiftGPU(std::vector<cv::KeyPoint>& feature_locations_2d,
                              std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& feature_locations_3d,
                              const cv::Mat& depth,
                              const sensor_msgs::CameraInfoConstPtr& cam_info,
                              std::vector<float>& descriptors_in, cv::Mat& descriptors_out)
{

  struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);

  double depth_scaling = ParameterServer::instance()->get<double>("depth_scaling_factor");
  float x,y;//temp point, 
  //principal point and focal lengths:
  float cx = cam_info->K[2]; //(cloud_msg->width >> 1) - 0.5f;
  float cy = cam_info->K[5]; //(cloud_msg->height >> 1) - 0.5f;
  float fx = 1.0f / cam_info->K[0]; 
  float fy = 1.0f / cam_info->K[4]; 
  cv::Point2f p2d;

  if(feature_locations_3d.size()){
    ROS_INFO("There is already 3D Information in the FrameInfo, clearing it");
    feature_locations_3d.clear();
  }

  std::list<int> featuresUsed;
  
  int index = -1;
  for(unsigned int i = 0; i < feature_locations_2d.size(); /*increment at end of loop*/){
    ++index;

    p2d = feature_locations_2d[i].pt;
    float Z;
    if(ParameterServer::instance()->get<bool>("use_feature_min_depth")){
      Z = getMinDepthInNeighborhood(depth, p2d, feature_locations_2d[i].size);
    } else {
      Z = depth.at<float>(p2d.y, p2d.x) * depth_scaling;
    }
    // Check for invalid measurements
    if (std::isnan (Z))
    {
      ROS_DEBUG("Feature %d has been extracted at NaN depth. Omitting", i);
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }
    x = (p2d.x - cx) * Z * fx;
    y = (p2d.y - cy) * Z * fy;

    feature_locations_3d.push_back(Eigen::Vector4f(x,y, Z, 1.0));
    featuresUsed.push_back(index);  //save id for constructing the descriptor matrix
    i++; //Only increment if no element is removed from vector
  }

  //create descriptor matrix
  int size = feature_locations_3d.size();
  descriptors_out = cv::Mat(size, 128, CV_32F);
  siftgpu_descriptors.resize(size * 128);
  for (int y = 0; y < size && featuresUsed.size() > 0; ++y) {
    int id = featuresUsed.front();
    featuresUsed.pop_front();

    for (int x = 0; x < 128; ++x) {
      descriptors_out.at<float>(y, x) = descriptors_in[id * 128 + x];
      siftgpu_descriptors[y * 128 + x] = descriptors_in[id * 128 + x];
    }
  }
  /*
  //create descriptor matrix
  int size = feature_locations_3d.size();
  descriptors_out = cv::Mat(size, 128, CV_32F);
  for (int y = 0; y < size && featuresUsed.size() > 0; ++y) {
    int id = featuresUsed.front();
    featuresUsed.pop_front();

    for (int x = 0; x < 128; ++x) {
      descriptors_out.at<float>(y, x) = descriptors_in[id * 128 + x];
    }
  }
  */

  clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}

void Node::projectTo3DSiftGPU(std::vector<cv::KeyPoint>& feature_locations_2d,
                              std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& feature_locations_3d,
                              const pointcloud_type::Ptr point_cloud, 
                              std::vector<float>& descriptors_in, cv::Mat& descriptors_out)
{

  struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);

  cv::Point2f p2d;

  if(feature_locations_3d.size()){
    ROS_INFO("There is already 3D Information in the FrameInfo, clearing it");
    feature_locations_3d.clear();
  }

  std::list<int> featuresUsed;

  int index = -1;
  for(unsigned int i = 0; i < feature_locations_2d.size(); /*increment at end of loop*/){
    ++index;

    p2d = feature_locations_2d[i].pt;
    point_type p3d = point_cloud->at((int) p2d.x,(int) p2d.y);

    // Check for invalid measurements
    if ( isnan(p3d.x) || isnan(p3d.y) || isnan(p3d.z))
    {
      ROS_DEBUG_NAMED(__FILE__, "Feature %d has been extracted at NaN depth. Omitting", i);
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }

    feature_locations_3d.push_back(Eigen::Vector4f(p3d.x, p3d.y, p3d.z, 1.0));
    featuresUsed.push_back(index);  //save id for constructing the descriptor matrix
    i++; //Only increment if no element is removed from vector
  }

  //create descriptor matrix
  int size = feature_locations_3d.size();
  descriptors_out = cv::Mat(size, 128, CV_32F);
  siftgpu_descriptors.resize(size * 128);
  for (int y = 0; y < size && featuresUsed.size() > 0; ++y) {
    int id = featuresUsed.front();
    featuresUsed.pop_front();

    for (int x = 0; x < 128; ++x) {
      descriptors_out.at<float>(y, x) = descriptors_in[id * 128 + x];
      siftgpu_descriptors[y * 128 + x] = descriptors_in[id * 128 + x];
    }
  }

  clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}
#endif

void Node::projectTo3D(std::vector<cv::KeyPoint>& feature_locations_2d,
                       std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& feature_locations_3d,
                       pointcloud_type::ConstPtr point_cloud)
{
  struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);

  cv::Point2f p2d;

  if(feature_locations_3d.size()){
    ROS_INFO("There is already 3D Information in the FrameInfo, clearing it");
    feature_locations_3d.clear();
  }

  for(unsigned int i = 0; i < feature_locations_2d.size(); /*increment at end of loop*/){
    p2d = feature_locations_2d[i].pt;
    if (p2d.x >= point_cloud->width || p2d.x < 0 ||
        p2d.y >= point_cloud->height || p2d.y < 0 ||
        std::isnan(p2d.x) || std::isnan(p2d.y)){ //TODO: Unclear why points should be outside the image or be NaN
      ROS_WARN_STREAM("Ignoring invalid keypoint: " << p2d); //Does it happen at all? If not, remove this code block
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }

    point_type p3d = point_cloud->at((int) p2d.x,(int) p2d.y);

    // Check for invalid measurements
    if ( isnan(p3d.x) || isnan(p3d.y) || isnan(p3d.z))
    {
      ROS_DEBUG_NAMED(__FILE__, "Feature %d has been extracted at NaN depth. Omitting", i);
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }

    feature_locations_3d.push_back(Eigen::Vector4f(p3d.x, p3d.y, p3d.z, 1.0));
    i++; //Only increment if no element is removed from vector
  }

  clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}

void Node::projectTo3D(std::vector<cv::KeyPoint>& feature_locations_2d,
                       std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& feature_locations_3d,
                       const cv::Mat& depth,
                       const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
  double depth_scaling = ParameterServer::instance()->get<double>("depth_scaling_factor");
  float x,y;//temp point, 
  //principal point and focal lengths:
  float cx = cam_info->K[2]; //(cloud_msg->width >> 1) - 0.5f;
  float cy = cam_info->K[5]; //(cloud_msg->height >> 1) - 0.5f;
  float fx = 1.0f / cam_info->K[0]; 
  float fy = 1.0f / cam_info->K[4]; 

  cv::Point2f p2d;

  if(feature_locations_3d.size()){
    ROS_INFO("There is already 3D Information in the FrameInfo, clearing it");
    feature_locations_3d.clear();
  }

  for(unsigned int i = 0; i < feature_locations_2d.size(); /*increment at end of loop*/){
    p2d = feature_locations_2d[i].pt;
    if (p2d.x >= depth.cols || p2d.x < 0 ||
        p2d.y >= depth.rows || p2d.y < 0 ||
        std::isnan(p2d.x) || std::isnan(p2d.y)){ //TODO: Unclear why points should be outside the image or be NaN
      ROS_WARN_STREAM("Ignoring invalid keypoint: " << p2d); //Does it happen at all? If not, remove this code block
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }
    float Z;
    if(ParameterServer::instance()->get<bool>("use_feature_min_depth")){
      Z = getMinDepthInNeighborhood(depth, p2d, feature_locations_2d[i].size);
    } else {
      Z = depth.at<float>(p2d.y, p2d.x) * depth_scaling;
    }
    // Check for invalid measurements
    if(std::isnan (Z))
    {
      ROS_DEBUG("Feature %d has been extracted at NaN depth. Omitting", i);
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }
    x = (p2d.x - cx) * Z * fx;
    y = (p2d.y - cy) * Z * fy;

    feature_locations_3d.push_back(Eigen::Vector4f(x,y, Z, 1.0));
    i++; //Only increment if no element is removed from vector
  }

  clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}

template<class CONTAINER>
void Node::computeInliersAndError(const CONTAINER& matches,
                                  const Eigen::Matrix4f& transformation,
                                  const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& origins,
                                  const std::vector<std::pair<float, float> > origins_depth_stats,
                                  const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& earlier,
                                  const std::vector<std::pair<float, float> > targets_depth_stats,
                                  std::vector<cv::DMatch>& inliers, //output var
                                  double& mean_error,
                                  std::vector<double>& errors,
                                  double squaredMaxInlierDistInM) const
{ //output var

  struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);

  inliers.clear();
  errors.clear();

  std::vector<std::pair<float,int> > dists;

  assert(matches.size() > 0);
  mean_error = 0.0;
  BOOST_FOREACH(const cv::DMatch& m, matches)
  {
    const Eigen::Vector4f& origin = origins[m.queryIdx];
    const Eigen::Vector4f& target = earlier[m.trainIdx];
    if(origin(2) == 0.0 || target(2) == 0.0){
       ROS_WARN_STREAM("Invalid point. Query Pt " << m.queryIdx << ":\n" << origin << "\nTarget Pt " << m.trainIdx << ":\n" << target);
       continue;
    }
    double mahal_dist = errorFunction2(origin, target, transformation);
    if(mahal_dist > squaredMaxInlierDistInM)
      continue; //ignore outliers
    if(!(mahal_dist >= 0.0)){
      ROS_WARN_STREAM("Mahalanobis_ML_Error: "<<mahal_dist);
      ROS_WARN_STREAM("Transformation for error !>= 0:\n" << transformation << "Matches: " << matches.size());
      continue;
    }
    inliers.push_back(m); //include inlier
    mean_error += mahal_dist;
    errors.push_back(mahal_dist );
  }

  if (inliers.size()<3){ //at least the samples should be inliers
    ROS_WARN_COND(inliers.size() > 3, "No inliers at all in %d matches!", (int)matches.size()); // only warn if this checks for all initial matches
    mean_error = 1e9;
  } else {
    mean_error /= inliers.size();
    mean_error = sqrt(mean_error);
  }
  if(!(mean_error>0)) ROS_DEBUG_STREAM("Transformation for mean error !> 0: " << transformation);
  if(!(mean_error>0)) ROS_DEBUG_STREAM(mean_error << " " << inliers.size());
  clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");

}

template<class CONTAINER>
Eigen::Matrix4f Node::getTransformFromMatchesUmeyama(const Node* earlier_node, CONTAINER matches) const 
{
  Eigen::Matrix<float, 3, Eigen::Dynamic> to(3,matches.size()), from(3,matches.size());
  typename CONTAINER::const_iterator it = matches.begin();
  for (int i = 0 ;it!=matches.end(); it++, i++) {
    int this_id    = it->queryIdx;
    int earlier_id = it->trainIdx;

    from.col(i) = this->feature_locations_3d_[this_id].head<3>();
    to.col(i) = earlier_node->feature_locations_3d_[earlier_id].head<3>();
  }
  Eigen::Matrix4f res = Eigen::umeyama(from, to, false);
  return res;
}

template<class CONTAINER>
Eigen::Matrix4f Node::getTransformFromMatches(const Node* earlier_node,
                                              const CONTAINER & matches,
                                              bool& valid, 
                                              const float max_dist_m) const 
{
  pcl::TransformationFromCorrespondences tfc;
  valid = true;
  std::vector<Eigen::Vector3f> t, f;

  BOOST_FOREACH(const cv::DMatch& m, matches)
  {
    Eigen::Vector3f from = this->feature_locations_3d_[m.queryIdx].head<3>();
    Eigen::Vector3f to = earlier_node->feature_locations_3d_[m.trainIdx].head<3>();

    //Validate that 3D distances are corresponding
    if (max_dist_m > 0) {  //storing is only necessary, if max_dist is given
      if(f.size() >= 1)
      {
        float delta_f = (from - f.back()).squaredNorm();//distance to the previous query point
        float delta_t = (to   - t.back()).squaredNorm();//distance from one to the next train point

        if ( abs(delta_f-delta_t) > max_dist_m * max_dist_m ) {
          valid = false;
          return Eigen::Matrix4f();
        }
      }
      f.push_back(from);
      t.push_back(to);    
    }

    tfc.add(from, to,1.0);// 1.0/(to(2)*to(2)));//the further, the less weight b/c of quadratic accuracy decay
  }

  // get relative movement from samples
  return tfc.getTransformation().matrix();
}


///Find transformation with largest support, RANSAC style.
///Return false if no transformation can be found
bool Node::getRelativeTransformationTo(const Node* earlier_node,
                                       std::vector<cv::DMatch>* initial_matches,
                                       Eigen::Matrix4f& resulting_transformation,
                                       float& rmse, 
                                       std::vector<cv::DMatch>& matches) const
{
  struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);

  assert(initial_matches != NULL);
  matches.clear();
  
  if(initial_matches->size() <= (unsigned int) ParameterServer::instance()->get<int>("min_matches")){
    ROS_INFO("Only %d feature matches between %d and %d (minimal: %i)",(int)initial_matches->size() , this->id_, earlier_node->id_, ParameterServer::instance()->get<int>("min_matches"));
    return false;
  }

  //unsigned int min_inlier_threshold = int(initial_matches->size()*0.2);
  unsigned int min_inlier_threshold = (unsigned int) ParameterServer::instance()->get<int>("min_matches");
  if(min_inlier_threshold > 0.75 * initial_matches->size()){
    ROS_WARN("Lowering min_inlier_threshold from %d to %d, because there are only %d matches to begin with", min_inlier_threshold, (int) (0.75 * initial_matches->size()), (int)initial_matches->size());
    min_inlier_threshold = 0.75 * initial_matches->size();
  }

  std::vector<cv::DMatch> inlier; //holds those feature correspondences that support the transformation
  double inlier_error; //all squared errors
  srand((long)std::clock());
  
  // a point is an inlier if it's no more than max_dist_m m from its partner apart
  const float max_dist_m = ParameterServer::instance()->get<double>("max_dist_for_inliers");
  const int ransac_iterations = ParameterServer::instance()->get<int>("ransac_iterations");
  std::vector<double> dummy;

  // best values of all iterations 
  double best_error = 1e6,  best_error_coarse = 1e6;
  unsigned int best_inlier = 0,  valid_iterations = 0;//, best_inlier_cnt = 0;
  unsigned int best_inlier_coarse = 0;

  Eigen::Matrix4f transformation;
  Eigen::Matrix4f transformationU;
  
  //INITIALIZATION STEP WITH RANDOM SAMPLES ###############################################
  const unsigned int sample_size = 3;// chose this many randomly from the correspondences:
  for (int n_iter = 0; n_iter < ransac_iterations; n_iter++) {
    //generate a map of samples. Using a map solves the problem of drawing a sample more than once
    std::set<cv::DMatch> sample_matches;
    while(sample_matches.size() < sample_size){
      int id = rand() % initial_matches->size();
      sample_matches.insert(initial_matches->at(id));
    }

    bool valid; // valid is false iff the sampled points clearly aren't inliers themself 
    //ROS_INFO_STREAM("TRANSFORMATIONS");
    transformation = getTransformFromMatches(earlier_node, sample_matches,valid,max_dist_m);
    //ROS_INFO_STREAM("tfc:\n" <<  transformation);
    //transformationU = getTransformFromMatchesUmeyama(earlier_node, sample_matches);
    //ROS_INFO_STREAM("Umeyama:\n" <<  transformationU);
    if (!valid) continue; // valid is false iff the sampled points aren't inliers themself 
    if(transformation!=transformation) continue; //Contains NaN
    //test whether samples are inliers (more strict than before)
    computeInliersAndError(sample_matches, transformation, 
                           this->feature_locations_3d_, 
                           this->feature_depth_stats_, 
                           earlier_node->feature_locations_3d_, 
                           earlier_node->feature_depth_stats_, 
                           inlier, inlier_error,  /*output*/
                           dummy, max_dist_m*max_dist_m); 
    
    ROS_DEBUG_NAMED(__FILE__, "Transformation from and for %u samples results in an error of %f and %i inliers.", sample_size, inlier_error, (int)inlier.size());
    if(inlier_error > 1000) continue; //most possibly a false match in the samples
/*
    //COARSE ESTIMATE TO THROW OUT SURE OUTLIERS
    computeInliersAndError(*initial_matches, transformationU, 
                           this->feature_locations_3d_, 
                           this->feature_depth_stats_, 
                           earlier_node->feature_locations_3d_, 
                           earlier_node->feature_depth_stats_, 
                           inlier, inlier_error, 
                           dummy, max_dist_m*max_dist_m*4); //use twice the distance (4x squared dist) to get more inliers for refinement
    ROS_INFO_NAMED("statistics", "Umeyama Transforma from %u samples results in an error of %f and %i inliers for all matches (%i).", sample_size, inlier_error, (int)inlier.size(), (int)initial_matches->size());
    */
    //COARSE ESTIMATE TO THROW OUT SURE OUTLIERS
    computeInliersAndError(*initial_matches, transformation, 
                           this->feature_locations_3d_, 
                           this->feature_depth_stats_, 
                           earlier_node->feature_locations_3d_, 
                           earlier_node->feature_depth_stats_, 
                           inlier, inlier_error,  /*output*/
                           dummy, max_dist_m*max_dist_m*4); //use twice the distance (4x squared dist) to get more inliers for refinement
    ROS_DEBUG_NAMED("statistics", "TFC Transformation from %u samples results in an error of %f and %i inliers for all matches (%i).", sample_size, inlier_error, (int)inlier.size(), (int)initial_matches->size());

    if(inlier.size() < min_inlier_threshold || inlier_error > max_dist_m){
      ROS_DEBUG_NAMED(__FILE__, "Skipped iteration: inliers: %i (min %i), inlier_error: %.2f (max %.2f)", (int)inlier.size(), (int) min_inlier_threshold,  inlier_error*100, max_dist_m*100);
      continue; //hopeless case
    }
    //Inferior to coarse estimates?
    if (inlier.size() < best_inlier_coarse && inlier_error > best_error_coarse) {
      continue; //hopeless case
    }
    best_inlier_coarse = inlier.size();
    best_error_coarse = inlier_error;
    //Totally superior?
    if (inlier.size() > best_inlier && inlier_error < best_inlier) {
      best_inlier = inlier.size();
      best_error = inlier_error;
    }

    ROS_DEBUG_NAMED(__FILE__, "Refining iteration from %i samples: all matches: %i, inliers: %i, inlier_error: %f", (int)sample_size, (int)initial_matches->size(), (int)inlier.size(), inlier_error);
    valid_iterations++;
    assert(inlier_error>=0);

    //Performance hacks:
    if (inlier.size() > initial_matches->size()*0.5) n_iter+=10;///Iterations with more than half of the initial_matches inlying, count twice
    if (inlier.size() > initial_matches->size()*0.8) n_iter+=20; ///Iterations with more than 80% of the initial_matches inlying, count threefold

    if (inlier_error < best_error) { //copy this to the result
      resulting_transformation = transformation;
      matches = inlier;
      assert(matches.size()>= min_inlier_threshold);
      //best_inlier_cnt = inlier.size();
      rmse = inlier_error;
      best_error = inlier_error;
    }


    //REFINEMENT STEP FROM INLIERS ###############################################
    double new_inlier_error;
    transformation = getTransformFromMatches(earlier_node, matches, valid); // compute new trafo from all inliers:
    if(transformation!=transformation) continue; //Contains NaN
    computeInliersAndError(*initial_matches, transformation,
                           this->feature_locations_3d_, 
                           this->feature_depth_stats_, 
                           earlier_node->feature_locations_3d_,
                           earlier_node->feature_depth_stats_, 
                           inlier, new_inlier_error, dummy, max_dist_m*max_dist_m);
    ROS_DEBUG_NAMED(__FILE__, "Refined Transformation from all matches (%i) results in an error of %f and %i inliers for all matches.", (int)initial_matches->size(), inlier_error, (int)inlier.size());

    if(inlier.size() < min_inlier_threshold || new_inlier_error > max_dist_m){
      continue;
    }
    //Totally superior?
    if (inlier.size() > best_inlier && new_inlier_error < best_inlier) {
      best_inlier = inlier.size();
      best_error = inlier_error;
      resulting_transformation = transformation;
      matches = inlier;
      assert(matches.size()>= min_inlier_threshold);
      //assert(matches.size()>= ((float)initial_matches->size())*min_inlier_ratio);
      rmse = new_inlier_error;
      best_error = new_inlier_error;
    }
  } //iterations

  ROS_INFO("%i good iterations (from %i), inlier pct %i, inlier cnt: %i, error: %.2f cm",valid_iterations, ransac_iterations, (int) (matches.size()*1.0/initial_matches->size()*100),(int) matches.size(),rmse*100);
  // ROS_INFO("best overall: inlier: %i, error: %.2f",best_inlier_invalid, best_error_invalid*100);

  clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
  bool enough_absolute = matches.size() >= min_inlier_threshold;
  return enough_absolute;
}


#ifdef USE_ICP_CODE
void Node::Eigen2GICP(const Eigen::Matrix4f& m, dgc_transform_t g_m){
  for(int i=0;i<4; i++)
    for(int j=0;j<4; j++)
      g_m[i][j] = m(i,j);

}
void Node::GICP2Eigen(const dgc_transform_t g_m, Eigen::Matrix4f& m){
  for(int i=0;i<4; i++)
    for(int j=0;j<4; j++)
      m(i,j) = g_m[i][j];
}

void Node::gicpSetIdentity(dgc_transform_t m){
  for(int i=0;i<4; i++)
    for(int j=0;j<4; j++)
      if (i==j)
        m[i][j] = 1;
      else
        m[i][j] = 0;
}
#endif



MatchingResult Node::matchNodePair(const Node* older_node)
{
  MatchingResult mr;
  bool found_transformation = false;
  if(initial_node_matches_ > ParameterServer::instance()->get<int>("max_connections")) return mr; //enough is enough
  const unsigned int min_matches = (unsigned int) ParameterServer::instance()->get<int>("min_matches");// minimal number of feature correspondences to be a valid candidate for a link
  // struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);

  this->featureMatching(older_node, &mr.all_matches); 

  ROS_DEBUG_NAMED(__FILE__, "found %i inital matches",(int) mr.all_matches.size());
  if (mr.all_matches.size() < min_matches){
      ROS_INFO("Too few inliers between %i and %i for RANSAC method. Only %i correspondences to begin with.",
               older_node->id_,this->id_,(int)mr.all_matches.size());
  } 
  else {//All good for feature based transformation estimation
      found_transformation = getRelativeTransformationTo(older_node,&mr.all_matches, mr.ransac_trafo, mr.rmse, mr.inlier_matches); 
      //Statistics
      float nn_ratio = 0.0;
      if(found_transformation){
        double w = 1.0 + (double)mr.inlier_matches.size()-(double)min_matches;///(double)mr.all_matches.size();
        for(unsigned int i = 0; i < mr.inlier_matches.size(); i++){
          nn_ratio += mr.inlier_matches[i].distance;
        }
        nn_ratio /= mr.inlier_matches.size();
        mr.final_trafo = mr.ransac_trafo;
        mr.edge.informationMatrix =   Eigen::Matrix<double,6,6>::Identity()*(w*w); //TODO: What do we do about the information matrix? Scale with inlier_count. Should rmse be integrated?)
        ROS_INFO("RANSAC found a valid transformation with %d inliers matches with average ratio %f",(int) mr.inlier_matches.size(), nn_ratio);
      } else {
        for(unsigned int i = 0; i < mr.all_matches.size(); i++){
          nn_ratio += mr.all_matches[i].distance;
        }
        nn_ratio /= mr.all_matches.size();
        ROS_WARN("RANSAC found no valid trafo, but had initially %d feature matches with average ratio %f",(int) mr.all_matches.size(), nn_ratio);
      }
  } 
  
#ifdef USE_ICP_CODE
  if(ParameterServer::instance()->get<bool>("use_icp")){
      if((int)this->id_ - (int)older_node->id_ >= 5){ //This is an "old node" that won't be compared via gicp anymore
        //clearGICPStructure();//save some memory, not too important though
      }
      else if(!found_transformation && initial_node_matches_ == 0) //no matches were found, and frames are not too far apart => identity is a good initial guess.
      {
          ROS_INFO("Falling back to GICP for Transformation between Nodes %d and %d",this->id_ , older_node->id_);
          if(getRelativeTransformationTo_ICP_code(older_node,mr.icp_trafo, mr.ransac_trafo) && //converged
             !((mr.icp_trafo.array() != mr.icp_trafo.array()).any())) //No NaNs
          {
              ROS_INFO("GICP Successful");
              found_transformation = true;
              mr.final_trafo = mr.icp_trafo;    
              mr.edge.informationMatrix = Eigen::Matrix<double,6,6>::Identity()*(1000); //TODO: What do we do about the information matrix? 
              //translation not as accurate as rotation
              mr.edge.informationMatrix(0,0) = 200;
              mr.edge.informationMatrix(1,1) = 200;
              mr.edge.informationMatrix(2,2) = 200;
          }
      }
  }
#endif  
      //TODO: This code is outdated. Adapt it to match USE_ICP_CODE section
#ifdef USE_ICP_BIN
  if(!found_transformation && (this->id_ - older_node->id_ < 5) && 
     initial_node_matches_ == 0 && ParameterServer::instance()->get<bool>("use_icp"))
  {//GICP Stuff only if no matches were found, but frames are not too far apart (s.t. the identity is a good initial guess.
      ROS_INFO("Falling back to GICP binary");
      // improve transformation by using the generalized ICP
      // std::clock_t starttime_gicp1 = std::clock();
      bool converged = getRelativeTransformationTo_ICP_bin(older_node,mr.icp_trafo, &mr.ransac_trafo);
      //ROS_INFO_STREAM("Paper: ICP1: " << ((std::clock()-starttime_gicp1*1.0) / (double)CLOCKS_PER_SEC));

      ROS_INFO("icp: inliers: %i", (int)mr.inlier_matches.size());
      if(converged){ 
        found_transformation = true; 
        mr.final_trafo = mr.ransac_trafo * mr.icp_trafo;

        std::vector<double> errors;
        double error;
        std::vector<cv::DMatch> inliers;
        // check if icp improves alignment:
        computeInliersAndError(mr.inlier_matches, mr.final_trafo,
                               this->feature_locations_3d_, 
                               this->feature_depth_stats_, 
                               older_node->feature_locations_3d_,
                               older_node->feature_depth_stats_, 
                               inliers, error, errors, 0.04*0.04); 

        for (uint i=0; i<errors.size(); i++)
        {
          ROS_INFO_COND("error: " << round(errors[i]*10000)/100 );
        }

        ROS_INFO_COND("error was: " << mr.rmse << " and is now: " << error );

        double roll, pitch, yaw, dist;
        mat2components(mr.ransac_trafo, roll, pitch, yaw, dist);
        ROS_INFO_COND("ransac: " << roll << " "<< pitch << " "<< yaw << "   "<< dist );

        mat2components(mr.icp_trafo, roll, pitch, yaw, dist);
        ROS_INFO_COND("icp: " << roll << " "<< pitch << " "<< yaw << "   "<< dist );

        mat2components(mr.final_trafo, roll, pitch, yaw, dist);
        ROS_INFO_COND("final: " << roll << " "<< pitch << " "<< yaw << "   "<< dist );
        ROS_INFO_COND("ransac: " << std::endl << mr.ransac_trafo );
        ROS_INFO_COND("icp: " << std::endl << mr.icp_trafo );
        ROS_INFO_COND("total: " << std::endl << mr.final_trafo );


        if (error > (mr.rmse+0.02))
        {
          ROS_WARN("#### icp-error is too large, ignoring the connection");
        }
        else
          mr.final_trafo = mr.ransac_trafo * mr.icp_trafo;    
      }
      ROS_INFO_COND(!converged, "ICP did not converge.");
      w = (double)mr.inlier_matches.size();///(double)mr.all_matches.size();
      mr.edge.informationMatrix =   Eigen::Matrix<double,6,6>::Identity()*(w*w); //TODO: What do we do about the information matrix? Scale with inlier_count. Should rmse be integrated?)
  }
#endif

  if(found_transformation) {
      ROS_INFO("Returning Valid Edge");
      ++initial_node_matches_; //trafo is accepted
      //This signals a valid result:
      mr.edge.id1 = older_node->id_;//and we have a valid transformation
      mr.edge.id2 = this->id_; //since there are enough matching features,
      mr.edge.mean = eigen2G2O(mr.final_trafo.cast<double>());//we insert an edge between the frames
  }
  return mr;
}

void Node::clearFeatureInformation(){
  //clear only points, by swapping data with empty vector (so mem really gets freed)
	std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > f_l_3d;  
  f_l_3d.swap(feature_locations_3d_);
	std::vector<cv::KeyPoint> f_l_2d; 
  f_l_2d.swap(feature_locations_2d_);
  feature_descriptors_.release();
}
void Node::addPointCloud(pointcloud_type::Ptr new_pc){
  pc_col = new_pc;
}
void Node::clearPointCloud(){
  //clear only points, by swapping data with empty vector (so mem really gets freed)
  pc_col->width = 0;
  pc_col->height = 0;
  pointcloud_type pc_empty;
  pc_empty.points.swap(pc_col->points);
}

/*TODO use this to discount features at depth jumps (or duplicate them -> sensed position + minimum position
void Node::computeKeypointDepthStats(const cv::Mat& depth_img, const std::vector<cv::KeyPoint> keypoints)
{
    ROS_INFO("Computing Keypoint Depth Statistics");
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
    BOOST_FOREACH(cv::KeyPoint kp, keypoints)
    { 
      int radius = kp.size/2;
      int left = kp.pt.x-radius;
      int top  = kp.pt.y-radius;
      double nearest=0.0, farthest=0.0;
      cv::Mat keypoint_neighbourhood(depth_img, cv::Rect(left, top, (int)kp.size, (int)kp.size));
      ROS_DEBUG("Nearest: %f, Farthest: %f", nearest, farthest);
      if(isnan(nearest)) nearest = 1.0;
      if(isnan(farthest)) farthest = 10.0;
      cv::minMaxLoc(keypoint_neighbourhood, &nearest, &farthest);
      feature_depth_stats_.push_back(std::make_pair(nearest, farthest)); 
    }
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}
*/

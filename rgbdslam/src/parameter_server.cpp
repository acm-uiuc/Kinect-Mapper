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
#include "parameter_server.h"

using namespace std;

ParameterServer* ParameterServer::_instance = NULL;

ParameterServer::ParameterServer() {
    pre = ros::this_node::getName();
    pre += "/config/";

    defaultConfig();
    getValues();
    checkValues();
}

ParameterServer* ParameterServer::instance() {
    if (_instance == NULL) {
        _instance = new ParameterServer();
    }
    return _instance;
}
void ParameterServer::defaultConfig() {
    // Input data settings
    config["topic_image_mono"]              = std::string("/camera/rgb/image_color");        // color or grayscale image ros topic
    config["camera_info_topic"]             = std::string("/camera/rgb/camera_info");        // required for backprojection if no pointcloud topic is given
    config["topic_image_depth"]             = std::string("/camera/depth/image");            // depth image ros topic
    config["topic_points"]                  = std::string("");                               // If omitted, xyz will be computed from depth image. 
    config["wide_topic"]                    = std::string("");                               // topics for stereo cam, e.g. /wide_stereo/left/image_mono
    config["wide_cloud_topic"]              = std::string("");                               // topics for stereo cam e.g. /wide_stereo/points2
    config["subscriber_queue_size"]         = static_cast<int> (3);                          // cache incoming data (carefully, RGB-D Clouds are 10MB each)
    config["drop_async_frames"]             = static_cast<bool> (false);                     // Check timestamps of depth and visual image, reject if not in sync 
    config["depth_scaling_factor"]          = static_cast<double> (1.0);                     //Some kinects have a wrongly scaled depth
    config["bagfile_name"]                  = std::string("");                               //read data from a bagfile, make sure to enter the right topics above
    config["data_skip_step"]                = static_cast<int> (1);                          // skip every n-th frame completely  

    // Output data settings
    config["store_pointclouds"]             = static_cast<bool> (true);                      // if the point clouds are not needed online, setting this to false saves lots of memory 
    config["individual_cloud_out_topic"]    = std::string("/rgbdslam/batch_clouds");         // Use this topic when sending the individual clouds with the computed transforms, e.g. for octomap_server
    config["aggregate_cloud_out_topic"]     = std::string("/rgbdslam/aggregate_clouds");     // Use this topic when sending the all points in one big registered cloud
    config["send_clouds_rate"]              = static_cast<double> (5);                       // When sending the point clouds (e.g. to RVIZ or Octomap Server) limit sending to this many clouds per second
    config["publisher_queue_size"]          = static_cast<int> (5);                          // ROS standard parameter for all publishers

    // TF information settings 
    config["fixed_frame_name"]              = std::string("/map");                           // The computed camera transforms are with respect to this frame. It is set to the identity for the first frame processed or, if ground truth is available, to the ground truth of the first frame
    config["odom_frame_name"]               = std::string("/odom");                          // A fixed frame estimation from somewhere else (e.g. odometry, laser-based mapping). Doesn't need to correspond to the pose of the fixed_frame_name
    config["ground_truth_frame_name"]       = std::string("");                               //use empty string if no ground truth tf frame available
    config["base_frame_name"]               = std::string("/openni_camera");                 //if the camera is articulated use robot base
    config["fixed_camera"]                  = static_cast<bool> (true);                      //is camera fixed relative to base?

    // Visual Features, to activate GPU-based features see CMakeLists.txt 
    config["feature_detector_type"]         = std::string("SURF");                           //SURF, SIFT, FAST, ... see misc.cpp
    config["feature_extractor_type"]        = std::string("SURF");                           //SURF or SIFT
    config["matcher_type"]                  = std::string("FLANN");                          //SIFTGPU or FLANN or BRUTEFORCE
    config["max_keypoints"]                 = static_cast<int> (2000);                       // Extract no more than this many keypoints 
    config["min_keypoints"]                 = static_cast<int> (500);
    config["min_matches"]                   = static_cast<int> (50);                         //don't try RANSAC if less than this many matches (if using SiftGPU and GLSL you should use max. 60 matches)
    config["sufficient_matches"]            = static_cast<int> (1e9);                        // Extract no less than this many only honored by the adjustable SURF and FAST features
    config["adjuster_max_iterations"]       = static_cast<int> (10);                         // If outside of bounds for max_kp and min_kp, retry this many times with adapted threshold
    config["use_feature_min_depth"]         = static_cast<bool>(true);                       // consider the nearest point in the neighborhood of the feature as its depth, as it will dominate the motion

    // Frontend settings 
    config["max_translation_meter"]         = static_cast<double> (1e10);                    // Sanity check for smooth motion.
    config["max_rotation_degree"]           = static_cast<int> (360);                        // Sanity check for smooth motion.
    config["min_translation_meter"]         = static_cast<double> (0.05);                    // frames with motion less than this, will be omitted 
    config["min_rotation_degree"]           = static_cast<int> (2.5);                        // frames with motion less than this, will be omitted 
    config["max_dist_for_inliers"]          = static_cast<double> (3);                       //Mahalanobis distance for matches to be considered inliers by ransac
    config["ransac_iterations"]             = static_cast<int> (1000);                       // these are fast, so high values are ok 
    config["max_connections"]               = static_cast<int> (15);                         // stop frame comparisons after this many succesfully found spation relations 
    config["geodesic_depth"]                = static_cast<int> (3);                          // For comparisons with neighbors, consider those with a graph distance (hop count) equal or below this value as neighbors of the direct predecessor
    config["predecessor_candidates"]        = static_cast<int> (10);                         // Compare Features to this many direct sequential predecessors
    config["neighbor_candidates"]           = static_cast<int> (10);                         // Compare Features to this many graph neighbours. Sample from the candidates
    config["min_sampled_candidates"]        = static_cast<int> (10);                         // Compare Features to this many uniformly sampled nodes for corrspondences 
    config["use_icp"]                       = static_cast<bool> (false);                     // Activate GICP Fallback. Ignored if GICP is not compiled in (see top of CMakeLists.txt) 
    config["gicp_max_cloud_size"]           = static_cast<int> (10000);                      // Subsample for increased speed
    //Backend
    config["optimizer_iterations"]          = static_cast<int> (1);                          // maximum of iterations in online operation (i.e., does not affect the final optimization in batch mode). Optimizer stops after convergence anyway
    config["optimizer_skip_step"]           = static_cast<int> (1);                          // optimize every n-th frame. Set high for offline operation 
    config["backend_solver"]                = std::string("cholmod");                        //Which solver to use in g2o for matrix inversion: "csparse" , "cholmod" or "pcg"

    // Visualization Settings 
    config["use_glwidget"]                  = static_cast<bool> (true);                      //3D view
    config["use_gui"]                       = static_cast<bool> (true);                      //GUI vs Headless Mode
    config["visualize_mono_depth_overlay"]  = static_cast<bool> (false);                     //Show Depth and Monochrome image as overlay in featureflow
    config["visualization_skip_step"]       = static_cast<int> (1);                          // draw only every nth pointcloud row and line, high values require higher squared_meshing_threshold 
    config["squared_meshing_threshold"]     = static_cast<double> (0.0009);                  // don't triangulate over depth jumps. Should be increased with increasing visualization_skip_step
    config["show_axis"]                     = static_cast<bool> (true);                      // do/don't visualize the pose graph in glwidget

    // Misc 
    config["start_paused"]                  = static_cast<bool> (true);
    config["batch_processing"]              = static_cast<bool> (false);                     //store results and close after bagfile has been processed
    config["concurrent_node_construction"]  = static_cast<bool> (true);                      // detect+extract features for new frame, while current frame is inserted into graph 
    config["concurrent_optimization"]       = static_cast<bool> (true);                      // detect+extract features for new frame, while current frame is inserted into graph 
    config["concurrent_edge_construction"]  = static_cast<bool> (true);                      // compare current frame to many predecessors in parallel. Note that SIFTGPU matcher and GICP are mutex'ed for thread-safety
    config["voxelfilter_size"]              = static_cast<double> (-1.0);                    // in meter voxefilter displayed and stored pointclouds, incompatible with use_glwidget=true. Set negative to disable
    config["nn_distance_ratio"]             = static_cast<double> (0.6);                     //Feature correspondence is valid if distance to nearest neighbour is smaller than this parameter times the distance to the 2nd neighbour. This needs to be 0.9-1.0 for SIFTGPU w/ FLANN, since SIFTGPU Features are normalized
    config["keep_all_nodes"]                = static_cast<bool> (false);                     //Keep nodes with "no motion" assumption if too few inliers
    config["min_time_reported"]             = static_cast<double> (1e9);                     //for easy profiling. by default, nothing should be reported
    config["preserve_raster_on_save"]       = static_cast<bool> (false);                     //Filter NaNs when saving clouds, destroying the image raster
}

void ParameterServer::getValues() {
    map<string, boost::any>::const_iterator itr;
    for (itr = config.begin(); itr != config.end(); ++itr) {
        string name = itr->first;
        if (itr->second.type() == typeid(string)) {
            config[name] = getFromParameterServer<string> (pre + name,
                    boost::any_cast<string>(itr->second));
            ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<string>(itr->second));
        } else if (itr->second.type() == typeid(int)) {
            config[name] = getFromParameterServer<int> (pre + name,
                    boost::any_cast<int>(itr->second));
            ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<int>(itr->second));
        } else if (itr->second.type() == typeid(double)) {
            config[name] = getFromParameterServer<double> (pre + name,
                    boost::any_cast<double>(itr->second));
            ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<double>(itr->second));
        } else if (itr->second.type() == typeid(bool)) {
            config[name] = getFromParameterServer<bool> (pre + name,
                    boost::any_cast<bool>(itr->second));
            ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<bool>(itr->second));
        }
    }
}

void ParameterServer::checkValues() {
    if (get<string>("matcher_type").compare("SIFTGPU") == 0
            && get<bool>("concurrent_node_construction") == true) {
        config["concurrent_node_construction"] = static_cast<bool>(false);
        ROS_ERROR("Cannot use concurrent node construction with SiftGPU matcher! 'concurrent_node_construction' was set to false");
    }

    if (get<string>("matcher_type").compare("SIFTGPU") == 0
            && get<bool>("concurrent_edge_construction") == true) {
        config["concurrent_edge_construction"] = static_cast<bool>(false);
        ROS_ERROR("Cannot use concurrent edge construction with SiftGPU matcher! 'concurrent_edge_construction' was set to false");
    }
}

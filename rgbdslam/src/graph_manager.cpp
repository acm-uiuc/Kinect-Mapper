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


#include <sys/time.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
//#include <rgbdslam/CloudTransforms.h>
#include "graph_manager.h"
#include "misc.h"
#include "pcl_ros/transforms.h"
#include "pcl/io/pcd_io.h"
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/features2d/features2d.hpp>
#include <QThread>
#include <qtconcurrentrun.h>
#include <QtConcurrentMap> 
#include <QFile>
#include <utility>
#include <fstream>
#include <limits>
#include <boost/foreach.hpp>

#include "g2o/math_groups/se3quat.h"
#include "g2o/types/slam3d/edge_se3_quat.h"

#include "g2o/core/block_solver.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"

//typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 3> >  SlamBlockSolver;
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearCSparseSolver;
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearCholmodSolver;
typedef g2o::LinearSolverPCG<SlamBlockSolver::PoseMatrixType> SlamLinearPCGSolver;
//typedef std::map<int, g2o::VertexSE3*> VertexIDMap;
typedef std::tr1::unordered_map<int, g2o::HyperGraph::Vertex*>     VertexIDMap;
typedef std::pair<int, g2o::HyperGraph::Vertex*> VertexIDPair;
//std::tr1::unordered_map<int, g2o::HyperGraph::Vertex* >
typedef std::set<g2o::HyperGraph::Edge*> EdgeSet;


GraphManager::GraphManager(ros::NodeHandle nh) :
    optimizer_(NULL), 
    latest_transform_(), //constructs identity
    reset_request_(false),
    marker_id_(0),
    last_matching_node_(-1),
    batch_processing_runs_(false),
    process_node_runs_(false),
    localization_only_(false),
    someone_is_waiting_for_me_(false),
    loop_closures_edges(0), sequential_edges(0),
    current_backend_("none")
{
  struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);

  ParameterServer* ps = ParameterServer::instance();
  createOptimizer(ps->get<std::string>("backend_solver"));

  batch_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(ps->get<std::string>("individual_cloud_out_topic"),
                                                            ps->get<int>("publisher_queue_size"));
  whole_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(ps->get<std::string>("aggregate_cloud_out_topic"),
                                                            ps->get<int>("publisher_queue_size"));
  ransac_marker_pub_ = nh.advertise<visualization_msgs::Marker>("/rgbdslam/correspondence_marker", 
                                                                ps->get<int>("publisher_queue_size"));
  marker_pub_ = nh.advertise<visualization_msgs::Marker>("/rgbdslam/pose_graph_markers",
                                                         ps->get<int>("publisher_queue_size"));
  computed_motion_ = tf::Transform::getIdentity();
  init_base_pose_  = tf::Transform::getIdentity();
  base2points_     = tf::Transform::getIdentity();
  //timer_ = nh.createTimer(ros::Duration(0.1), &GraphManager::broadcastTransform, this);
  Max_Depth = -1;

  clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}

void GraphManager::createOptimizer(std::string backend, g2o::SparseOptimizer* optimizer)
{
  QMutexLocker locker(&optimizer_mutex);
  // allocating the optimizer
  if(optimizer == NULL){
    delete optimizer_; 
    optimizer_ = new g2o::SparseOptimizer();
    optimizer_->setVerbose(true);
  } else if (optimizer_ != optimizer){
    delete optimizer_; 
    optimizer_ = new g2o::SparseOptimizer();
    optimizer_->setVerbose(true);
  } 
  SlamBlockSolver* solver = NULL;
  if(backend == "cholmod" || backend == "auto"){
    SlamLinearCholmodSolver* linearSolver = new SlamLinearCholmodSolver();
    linearSolver->setBlockOrdering(false);
    solver = new SlamBlockSolver(optimizer_, linearSolver);
    current_backend_ = "cholmod";
  }
  else if(backend == "csparse"){
    SlamLinearCSparseSolver* linearSolver = new SlamLinearCSparseSolver();
    linearSolver->setBlockOrdering(false);
    solver = new SlamBlockSolver(optimizer_, linearSolver);
    current_backend_ = "csparse";
  }
  else if(backend == "pcg"){
    SlamLinearPCGSolver* linearSolver = new SlamLinearPCGSolver();
    solver = new SlamBlockSolver(optimizer_, linearSolver);
    current_backend_ = "pcg";
  }
  else {
    ROS_ERROR("Bad Parameter for g2o Solver backend: %s. User cholmod, csparse or pcg", backend.c_str());
    ROS_INFO("Falling Back to Cholmod Solver");
    SlamLinearCholmodSolver* linearSolver = new SlamLinearCholmodSolver();
    linearSolver->setBlockOrdering(false);
    solver = new SlamBlockSolver(optimizer_, linearSolver);
    current_backend_ = "cholmod";
  }
  optimizer_->setSolver(solver);
}

//WARNING: Dangerous
void GraphManager::deleteFeatureInformation() {
  ROS_WARN("Clearing out Feature information from nodes");
  //Q_FOREACH(Node* node, graph_) {
  BOOST_FOREACH(GraphNodeType entry, graph_){
    entry.second->clearFeatureInformation();
  }
}

GraphManager::~GraphManager() {
  //TODO: delete all Nodes
    //for (unsigned int i = 0; i < optimizer_->vertices().size(); ++i) {
    //Q_FOREACH(Node* node, graph_) { delete node; }
    BOOST_FOREACH(GraphNodeType entry, graph_) { 
      delete entry.second; 
    }
    graph_.clear();
    delete (optimizer_);
    ransac_marker_pub_.shutdown();
    whole_cloud_pub_.shutdown();
    marker_pub_.shutdown();
    batch_cloud_pub_.shutdown();

}

void GraphManager::drawFeatureFlow(cv::Mat& canvas, cv::Scalar line_color,
                                   cv::Scalar circle_color){
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
    if(!ParameterServer::instance()->get<bool>("use_gui")){ return; }
    ROS_DEBUG("Number of features to draw: %d", (int)last_inlier_matches_.size());

    const double pi_fourth = 3.14159265358979323846 / 4.0;
    const int line_thickness = 1;
    const int circle_radius = 6;
    const int CV_AA = 16;
    if(graph_.empty()) {
      ROS_WARN("Feature Flow for empty graph requested. Bug?");
      return;
    } else if(graph_.size() == 1 || last_matching_node_ == -1 ) {//feature flow is only available between at least two nodes
      Node* newernode = graph_[graph_.size()-1];
      cv::drawKeypoints(canvas, newernode->feature_locations_2d_, canvas, cv::Scalar(255), 5);
      return;
    } 

    Node* earliernode = graph_[last_matching_node_];//graph_.size()-2; //compare current to previous
    Node* newernode = graph_[graph_.size()-1];
    if(earliernode == NULL){
      if(newernode == NULL ){ ROS_ERROR("Nullpointer for Node %u", (unsigned int)graph_.size()-1); }
      ROS_ERROR("Nullpointer for Node %d", last_matching_node_);
      last_matching_node_ = 0;
      return;
    } else if(newernode == NULL ){
      ROS_ERROR("Nullpointer for Node %u", (unsigned int)graph_.size()-1);
      return;
    }

    //encircle all keypoints in this image
    //for(unsigned int feat = 0; feat < newernode->feature_locations_2d_.size(); feat++) {
    //    cv::Point2f p; 
    //    p = newernode->feature_locations_2d_[feat].pt;
    //    cv::circle(canvas, p, circle_radius, circle_color, line_thickness, 8);
    //}
    cv::Mat tmpimage = cv::Mat::zeros(canvas.rows, canvas.cols, canvas.type());
    cv::drawKeypoints(canvas, newernode->feature_locations_2d_, tmpimage, circle_color, 5);
    canvas+=tmpimage;
    for(unsigned int mtch = 0; mtch < last_inlier_matches_.size(); mtch++) {
        cv::Point2f p,q; //TODO: Use sub-pixel-accuracy
        unsigned int newer_idx = last_inlier_matches_[mtch].queryIdx;
        unsigned int earlier_idx = last_inlier_matches_[mtch].trainIdx;
        q = newernode->feature_locations_2d_[newer_idx].pt;
        p = earliernode->feature_locations_2d_[earlier_idx].pt;

        double angle;    angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
        double hypotenuse = cv::norm(p-q);
            cv::line(canvas, p, q, line_color, line_thickness, CV_AA);
        if(hypotenuse > 1.5){  //only larger motions larger than one pix get an arrow tip
            cv::line( canvas, p, q, line_color, line_thickness, CV_AA );
            /* Now draw the tips of the arrow.  */
            p.x =  (q.x + 4 * cos(angle + pi_fourth));
            p.y =  (q.y + 4 * sin(angle + pi_fourth));
            cv::line( canvas, p, q, line_color, line_thickness, CV_AA );
            p.x =  (q.x + 4 * cos(angle - pi_fourth));
            p.y =  (q.y + 4 * sin(angle - pi_fourth));
            cv::line( canvas, p, q, line_color, line_thickness, CV_AA );
        } else { //draw a smaller circle into the bigger one 
            cv::circle(canvas, p, circle_radius-2, circle_color, line_thickness, CV_AA);
        }
    }
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}


QList<int> GraphManager::getPotentialEdgeTargetsWithDijkstra(const Node* new_node, int sequential_targets, int geodesic_targets, int sampled_targets, int predecessor_id)
{
    QList<int> ids_to_link_to; //return value
    if(predecessor_id < 0) predecessor_id = graph_.size()-1;
    //output only loop
    std::stringstream ss;
    ss << "Node ID's to compare with candidate for node " << graph_.size() << ". Sequential: ";

    if((int)optimizer_->vertices().size() <= sequential_targets+geodesic_targets+sampled_targets ||
       optimizer_->vertices().size() <= 1)
    { //if less prev nodes available than targets requestet, just use all
      sequential_targets = sequential_targets+geodesic_targets+sampled_targets;
      geodesic_targets = 0;
      sampled_targets = 0;
      predecessor_id = graph_.size()-1;
    }

    if(sequential_targets > 0){
      //all the sequential targets (will be checked last)
      for (int i=1; i < sequential_targets+1 && predecessor_id-i >= 0; i++) { 
          ids_to_link_to.push_back(predecessor_id-i); 
          ss << ids_to_link_to.back() << ", " ; 
      }
    }

    if(geodesic_targets > 0){
      g2o::HyperDijkstra hypdij(optimizer_);
      g2o::UniformCostFunction cost_function;
      g2o::VertexSE3* prev_vertex = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(predecessor_id));
      hypdij.shortestPaths(prev_vertex,&cost_function,3);
      g2o::HyperGraph::VertexSet& vs = hypdij.visited();


      //Geodesic Neighbours except sequential
      std::map<int,int> neighbour_indices; //maps neighbour ids to their weights in sampling
      int sum_of_weights=0;
      for (g2o::HyperGraph::VertexSet::iterator vit=vs.begin(); vit!=vs.end(); vit++) { //FIXME: Mix of vertex id and graph node (with features) id
        int id = (*vit)->id();
        if(id < predecessor_id-sequential_targets || (id > predecessor_id && id <= (int)graph_.size()-1)){ //Geodesic Neighbours except sequential 
            int weight = abs(predecessor_id-id);
            neighbour_indices[id] = weight; //higher probability to be drawn if far away
            sum_of_weights += weight;
        }
      }

      //Sample targets from graph-neighbours
      ss << "Dijkstra: ";
      while(ids_to_link_to.size() < sequential_targets+geodesic_targets && neighbour_indices.size() != 0){ 
        int random_pick = rand() % sum_of_weights;
        ROS_DEBUG("Pick: %d/%d", random_pick, sum_of_weights);
        int weight_so_far = 0;
        for(std::map<int,int>::iterator map_it = neighbour_indices.begin(); map_it != neighbour_indices.end(); map_it++ ){
          weight_so_far += map_it->second;
          ROS_DEBUG("Checking: %d, %d, %d", map_it->first, map_it-> second, weight_so_far);
          if(weight_so_far > random_pick){//found the selected one
            int sampled_id = map_it->first;
            ids_to_link_to.push_front(sampled_id);
            ss << ids_to_link_to.front() << ", " ; 
            sum_of_weights -= map_it->second;
            ROS_DEBUG("Taking ID: %d, decreasing sum of weights to %d", map_it->first, sum_of_weights);
            neighbour_indices.erase(map_it);
            ROS_ERROR_COND(sum_of_weights<0, "Sum of weights should never be zero");
            break;
          }
          ROS_DEBUG("Skipping ID: %d", map_it->first);
        }//for
      }
    }
    
    if(sampled_targets > 0){
      ss << "Random Sampling: ";
      std::vector<int> non_neighbour_indices;//initially holds all, then neighbours are deleted
      non_neighbour_indices.reserve(graph_.size());
      for (QList<int>::iterator it = keyframe_ids_.begin(); it != keyframe_ids_.end(); it++){
        if(ids_to_link_to.contains(*it) == 0){
          non_neighbour_indices.push_back(*it); 
        }
      }

      //Sample targets from non-neighbours (search new loops)
      while(ids_to_link_to.size() < geodesic_targets+sampled_targets+sequential_targets && non_neighbour_indices.size() != 0){ 
          int index_of_v_id = rand() % non_neighbour_indices.size();
          int sampled_id = non_neighbour_indices[index_of_v_id];
          non_neighbour_indices[index_of_v_id] = non_neighbour_indices.back(); //copy last id to position of the used id
          non_neighbour_indices.resize(non_neighbour_indices.size()-1); //drop last id
          ids_to_link_to.push_front(sampled_id);
          ss << ids_to_link_to.front() << ", " ; 
      }
    }

    ROS_INFO("%s", ss.str().c_str());
    return ids_to_link_to; //only compare to first frame
}

QList<int> GraphManager::getPotentialEdgeTargets(const Node* new_node, int last_targets, int sample_targets)
{
    int gsize = graph_.size();
    ROS_ERROR_COND(gsize == 0, "Do not call this function as long as the graph is empty");
    QList<int> ids_to_link_to; //return value

    //Special Cases of max_targets argument
    if(last_targets + sample_targets <= 0) {        //Negative: No comparisons
        return ids_to_link_to;
    }

    //All the last few nodes
    if(gsize <= sample_targets+last_targets){
      last_targets = gsize;//Can't compare to more nodes than exist
      sample_targets = 0;
    }
    for(int i = 2; i <= gsize && i <= last_targets; i++){//start at two, b/c the prev node is always already checked in addNode
        ids_to_link_to.push_back(gsize-i);
    }


    while(ids_to_link_to.size() < sample_targets+last_targets && ids_to_link_to.size() < gsize-1){ 
        int sample_id = rand() % (gsize - 1);
        ROS_DEBUG_STREAM("Sample: " << sample_id << " Graph size: " << gsize << " ids_to_link_to.size: " << ids_to_link_to.size());
        QList<int>::const_iterator i1 = qFind(ids_to_link_to, sample_id);
        if(i1 != ids_to_link_to.end()) 
          continue;
        ids_to_link_to.push_back(sample_id);
    }


    //output only loop
    std::stringstream ss;
    ss << "Node ID's to compare with candidate for node " << graph_.size() << ":";
    for(int i = 0; i < (int)ids_to_link_to.size(); i++){
        ss << ids_to_link_to[i] << ", " ; 
    }
    ROS_INFO("%s", ss.str().c_str());
    return ids_to_link_to;
}

void GraphManager::resetGraph(){
    marker_id_ =0;
    ParameterServer* ps = ParameterServer::instance();
    createOptimizer(ps->get<std::string>("backend_solver"));

    //Q_FOREACH(Node* node, graph_) { delete node; }
    BOOST_FOREACH(GraphNodeType entry, graph_){ delete entry.second; }
    //for(unsigned int i = 0; i < graph_.size(); delete graph_[i++]);//No body
    graph_.clear();
    keyframe_ids_.clear();
    Q_EMIT resetGLViewer();
    last_matching_node_ = -1;
    latest_transform_.setToIdentity();
    current_poses_.clear();
    current_edges_.clear();
    reset_request_ = false;
    loop_closures_edges = 0; 
    sequential_edges = 0;
}
/*NEW
void GraphManager::addOutliers(Node* new_node, std::vector<cv::DMatch> inlier_matches){
    std::vector<bool> outlier_flags(new_node->feature_locations_3d_.size(), true);
    BOOST_FOREACH(cv::DMatch& m, inlier_matches){ 
      outlier_flags[m.queryIdx] = false;

}
*/
void GraphManager::firstNode(Node* new_node) 
{
    init_base_pose_ =  new_node->getGroundTruthTransform();//identity if no MoCap available
    printTransform("Ground Truth Transform for First Node", init_base_pose_);
    new_node->buildFlannIndex(); // create index so that next nodes can use it
    graph_[new_node->id_] = new_node;
    g2o::VertexSE3* reference_pose = new g2o::VertexSE3;
    reference_pose->setId(0);
    g2o::SE3Quat g2o_ref_se3 = tf2G2O(init_base_pose_);
    reference_pose->setEstimate(g2o_ref_se3);
    reference_pose->setFixed(true);//fix at origin
    optimizer_mutex.lock();
    optimizer_->addVertex(reference_pose); 
    optimizer_mutex.unlock();
    QString message;
    Q_EMIT setGUIInfo(message.sprintf("Added first node with %i keypoints to the graph", (int)new_node->feature_locations_2d_.size()));
    //pointcloud_type::Ptr the_pc(new_node->pc_col); //this would delete the cloud after the_pc gets out of scope
    latest_transform_ = g2o2QMatrix(g2o_ref_se3);
    printQMatrix4x4("Latest Transform", latest_transform_);
    Q_EMIT setPointCloud(new_node->pc_col.get(), latest_transform_);
    current_poses_.append(latest_transform_);
    ROS_DEBUG("GraphManager is thread %d, New Node is at (%p, %p)", (unsigned int)QThread::currentThreadId(), new_node, graph_[0]);
    keyframe_ids_.push_back(new_node->id_);

    process_node_runs_ = false;
}

// returns true, iff node could be added to the cloud
bool GraphManager::addNode(Node* new_node) {
    /// \callergraph
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
    process_node_runs_ = true;

    last_inlier_matches_.clear();
    if(reset_request_) resetGraph(); 
    ParameterServer* ps = ParameterServer::instance();
    if ((int)new_node->feature_locations_2d_.size() < ps->get<int>("min_matches") && 
        ! ps->get<bool>("keep_all_nodes"))
    {
        ROS_DEBUG("found only %i features on image, node is not included",(int)new_node->feature_locations_2d_.size());
        process_node_runs_ = false;
        return false;
    }

    //set the node id only if the node is actually added to the graph
    //needs to be done here as the graph size can change inside this function
    new_node->id_ = graph_.size();

    //First Node, so only build its index, insert into storage and add a
    //vertex at the origin, of which the position is very certain
    if (graph_.size()==0){
        firstNode(new_node);
        return true;
    }

    unsigned int num_edges_before = optimizer_->edges().size(); 
    bool edge_to_keyframe = false;

    ROS_DEBUG("Graphsize: %d Nodes", (int) graph_.size());
    marker_id_ = 0; //overdraw old markers
    //last_matching_node_ = -1;


    //Node* prev_frame = graph_[graph_.size()-1];
    //if(localization_only_ && last_matching_node_ > 0){ prev_frame =  graph_[last_matching_node_]; }

    //Odometry Stuff
    int sequentially_previous_id = graph_.rbegin()->second->id_; 
    tf::StampedTransform odom_tf_new = new_node->getOdomTransform();
    tf::StampedTransform odom_tf_old = graph_[sequentially_previous_id]->getOdomTransform();
    tf::Transform odom_delta_tf = odom_tf_new * odom_tf_old.inverse();
    printTransform("Odometry Delta", odom_delta_tf);
    if(odom_tf_old.frame_id_ == "missing_odometry" || odom_tf_new.frame_id_ == "missing_odometry"){
      ROS_WARN("No Valid Odometry, using identity");
      odom_delta_tf = tf::Transform::getIdentity();
    }
    //int best_match_candidate_id = sequentially_previous_id; 

    //Initial Comparison ######################################################################
    //First check if trafo to last frame is big
    //Node* prev_frame = graph_[graph_.size()-1];
    Node* prev_frame = graph_[graph_.size()-1];
    if(localization_only_ && last_matching_node_ > 0){ prev_frame =  graph_[last_matching_node_]; }
    last_matching_node_ = -1; //Reset this, so after the first comparison, we know the last_matching_node_ is from this call of this method
    ROS_INFO("Comparing new node (%i) with previous node %i", new_node->id_, prev_frame->id_);
    MatchingResult mr = new_node->matchNodePair(prev_frame);
    if(mr.edge.id1 >= 0 && !isBigTrafo(mr.edge.mean)){
        ROS_WARN("Transformation not relevant. Did not add as Node");
        //Send the current pose via tf nevertheless
        tf::Transform incremental = g2o2TF(mr.edge.mean);
        g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(prev_frame->id_));
        tf::Transform previous = g2o2TF(v->estimate());
        tf::Transform combined = previous*incremental;
        broadcastTransform(new_node, combined);
        process_node_runs_ = false;
        return false;
    } else if(mr.edge.id1 >= 0){

        //mr.edge.informationMatrix *= geodesicDiscount(hypdij, mr); 
        ROS_DEBUG_STREAM("Information Matrix for Edge (" << mr.edge.id1 << "<->" << mr.edge.id2 << "\n" << mr.edge.informationMatrix);

        if (addEdgeToG2O(mr.edge, true, true)) {
            if(keyframe_ids_.contains(mr.edge.id1)) edge_to_keyframe = true;
            ROS_INFO("Added Edge between %i and %i. Inliers: %i",mr.edge.id1,mr.edge.id2,(int) mr.inlier_matches.size());
            last_matching_node_ = mr.edge.id1;
            last_inlier_matches_ = mr.inlier_matches;
            last_matches_ = mr.all_matches;
            edge_to_previous_node_ = mr.edge.mean;
            //addOutliers(Node* new_node, mr.inlier_matches);

        } else {
          process_node_runs_ = false;
          return false;
        }
    }
    //Eigen::Matrix4f ransac_trafo, final_trafo;
    QList<int> vertices_to_comp;
    int  seq_cand = localization_only_ ? 0 : ps->get<int>("predecessor_candidates");
    int geod_cand = ps->get<int>("neighbor_candidates");
    int samp_cand = ps->get<int>("min_sampled_candidates");
    vertices_to_comp = getPotentialEdgeTargetsWithDijkstra(new_node, seq_cand, geod_cand, samp_cand, last_matching_node_); 
    QList<const Node* > nodes_to_comp;//only necessary for parallel computation

    //MAIN LOOP: Compare node pairs ######################################################################
    if (ps->get<bool>("concurrent_edge_construction")) {
        for (int id_of_id = (int) vertices_to_comp.size() - 1; id_of_id >= 0; id_of_id--) {
            //First compile a qlist of the nodes to be compared, then run the comparisons in parallel,
            //collecting a qlist of the results (using the blocking version of mapped).
            nodes_to_comp.push_front(graph_[vertices_to_comp[id_of_id]]); 
        }
        QThreadPool* qtp = QThreadPool::globalInstance();
        ROS_INFO("Running node comparisons in parallel in %i (of %i) available threads", qtp->maxThreadCount() - qtp->activeThreadCount(), qtp->maxThreadCount());
        if (qtp->maxThreadCount() - qtp->activeThreadCount() == 1) {
            ROS_WARN("Few Threads Remaining: Increasing maxThreadCount to %i", qtp->maxThreadCount()+1);
            qtp->setMaxThreadCount(qtp->maxThreadCount() + 1);
        }
        QList<MatchingResult> results = QtConcurrent::blockingMapped(nodes_to_comp, boost::bind(&Node::matchNodePair, new_node, _1));

        /*
        while(!result_future.isFinished()){
          if(result_future.resultCount() > geod_cand+samp_cand && someone_is_waiting_for_me_){
            result_future.cancel();
            result_future.waitForFinished();//no dangling comparison that clash with the next node's comparison
            ROS_INFO("Enough comparisons");
            break;
          }
        }
        QList<MatchingResult> results = result_future.results();
        */

        for (int i = 0; i < results.size(); i++) {
            MatchingResult& mr = results[i];

            if (mr.edge.id1 >= 0) {
                //mr.edge.informationMatrix *= geodesicDiscount(hypdij, mr);
                ROS_INFO_STREAM("Information Matrix for Edge (" << mr.edge.id1 << "<->" << mr.edge.id2 << "\n" << mr.edge.informationMatrix);

                if (addEdgeToG2O(mr.edge, isBigTrafo(mr.edge.mean), mr.inlier_matches.size() > last_inlier_matches_.size())) 
                { 
                    ROS_INFO("Added Edge between %i and %i. Inliers: %i",mr.edge.id1,mr.edge.id2,(int) mr.inlier_matches.size());
                    if (mr.inlier_matches.size() > last_inlier_matches_.size()) {
                        last_matching_node_ = mr.edge.id1;
                        last_inlier_matches_ = mr.inlier_matches;
                        last_matches_ = mr.all_matches;
                        //last_edge_ = mr.edge.mean;
                    }
                    if(keyframe_ids_.contains(mr.edge.id1)) edge_to_keyframe = true;
                }
            }
        }
    } else {
        for (int id_of_id = (int) vertices_to_comp.size() - 1; id_of_id >= 0; id_of_id--) {
            Node* node_to_compare = graph_[vertices_to_comp[id_of_id]];
            ROS_INFO("Comparing new node (%i) with node %i / %i", new_node->id_, vertices_to_comp[id_of_id], node_to_compare->id_);
            MatchingResult mr = new_node->matchNodePair(node_to_compare);

            if (mr.edge.id1 >= 0) {
                //mr.edge.informationMatrix *= geodesicDiscount(hypdij, mr);
                ROS_INFO_STREAM("Information Matrix for Edge (" << mr.edge.id1 << "<->" << mr.edge.id2 << "\n" << mr.edge.informationMatrix);

                if (addEdgeToG2O(mr.edge, isBigTrafo(mr.edge.mean), mr.inlier_matches.size() > last_inlier_matches_.size())) 
                {
                    ROS_INFO("Added Edge between %i and %i. Inliers: %i",mr.edge.id1,mr.edge.id2,(int) mr.inlier_matches.size());
                    if (mr.inlier_matches.size() > last_inlier_matches_.size()) {
                        last_matching_node_ = mr.edge.id1;
                        last_inlier_matches_ = mr.inlier_matches;
                        last_matches_ = mr.all_matches;
                        //last_edge_ = mr.edge.mean;
                    }
                    if(keyframe_ids_.contains(mr.edge.id1)) edge_to_keyframe = true;
                }
            }
        }
    }

    //END OF MAIN LOOP: Compare node pairs ######################################################################
    bool found_trafo = (optimizer_->edges().size() != num_edges_before);

    if((found_trafo
        && !ps->get<std::string>("odom_frame_name").empty()
        && !(odom_tf_old.frame_id_ == "missing_odometry" 
             || odom_tf_new.frame_id_ == "missing_odometry")) 
       || (!found_trafo && ps->get<bool>("keep_all_nodes")))
    {
      ROS_INFO("Adding odometry motion edge for Node %i (if available, otherwise using identity)", (int)graph_.rbegin()->second->id_);
      LoadedEdge3D odom_edge;
      odom_edge.id2 = sequentially_previous_id;
      odom_edge.id1 = new_node->id_;
      odom_edge.mean = tf2G2O(odom_delta_tf);
      if(!found_trafo){
        latest_transform_ = g2o2QMatrix(odom_edge.mean);
        last_matching_node_ = odom_edge.id1;
        odom_edge.informationMatrix = Eigen::Matrix<double,6,6>::Identity()*(1); 
      }
      else {
        //FIXME get odometry information matrix and transform it to the optical frame
        odom_edge.informationMatrix = Eigen::Matrix<double,6,6>::Identity()*(10); 
        odom_edge.informationMatrix(1,1) = 100000000000000;
        odom_edge.informationMatrix(3,3) = 100000000000000;
        odom_edge.informationMatrix(5,5) = 100000000000000;
      }
      addEdgeToG2O(odom_edge, true,true) ;
    }
      


    if (optimizer_->edges().size() > num_edges_before) { //Success
        if(localization_only_)
        {
          optimizeGraph();
          broadcastTransform(new_node, computed_motion_);
          g2o::VertexSE3* new_v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(new_node->id_)); //FIXME: Mix of ids
          optimizer_->removeVertex(new_v); //Also removes the edges
        }
        else //localization_only_ == false
        {
          new_node->buildFlannIndex();
          graph_[new_node->id_] = new_node; //Node->id_ == Graph_ Index
          if(!edge_to_keyframe) {
            std::stringstream ss;
            ss << "Keyframes: ";
            BOOST_FOREACH(int id, keyframe_ids_){ ss << id << ", "; }
            ROS_INFO("%s", ss.str().c_str());
            keyframe_ids_.push_back(new_node->id_-1); //use the one before, because that one is still localized w.r.t. a keyframe
          }
          ROS_INFO("Added Node, new graphsize: %i nodes", (int) graph_.size());
          if((optimizer_->vertices().size() % ps->get<int>("optimizer_skip_step")) == 0){ 
            optimizeGraph();
          } else {
            current_poses_.append(latest_transform_);
            current_edges_.append( qMakePair((int)new_node->id_, last_matching_node_));
            //Q_EMIT setGraphEdges(new QList<QPair<int, int> >(current_edges_));
            //Q_EMIT updateTransforms(new QList<QMatrix4x4>(current_poses_));
          }
          //Q_EMIT updateTransforms(getAllPosesAsMatrixList());
          //Q_EMIT setGraphEdges(getGraphEdges());
          //make the transform of the last node known
          broadcastTransform(new_node, computed_motion_);
          visualizeGraphEdges();
          visualizeGraphNodes();
          visualizeFeatureFlow3D(marker_id_++);
          //if(last_matching_node_ <= 0){ cloudRendered(new_node->pc_col.get());}//delete points of non-matching nodes. They shall not be rendered

          //The following updates the 3D visualization. Important only if optimizeGraph is not called every frame, as that does the update too
          //pointcloud_type::Ptr the_pc = new_node->pc_col;
          pointcloud_type* cloud_to_visualize = new_node->pc_col.get();
          //if(!found_trafo) cloud_to_visualize = new pointcloud_type();
          Q_EMIT setPointCloud(cloud_to_visualize, latest_transform_);
        } 
    }else{ 
        if(graph_.size() == 1){//if there is only one node which has less features, replace it by the new one
          ROS_WARN("Choosing new initial node, because it has more features");
          if(new_node->feature_locations_2d_.size() > graph_[0]->feature_locations_2d_.size()){
            this->resetGraph();
            process_node_runs_ = false;
            return this->addNode(new_node);
          }
        } else { //delete new_node; //is now  done by auto_ptr
          ROS_WARN("Did not add as Node");
        }
    }
    QString message;
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
    Q_EMIT setGUIInfo(message.sprintf("%s, Graph Size: %iN/%iE, Duration: %f, Inliers: %i, &chi;<sup>2</sup>: %f", 
                                     (optimizer_->edges().size() > num_edges_before) ? "Added" : "Ignored",
                                     (int)optimizer_->vertices().size(), (int)optimizer_->edges().size(),
                                     elapsed, (int)last_inlier_matches_.size(), optimizer_->chi2()));
    process_node_runs_ = false;
    someone_is_waiting_for_me_ = false;
    return (optimizer_->edges().size() > num_edges_before);
}


void GraphManager::visualizeFeatureFlow3D(unsigned int marker_id, bool draw_outlier) const
{
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
    if (ransac_marker_pub_.getNumSubscribers() > 0){ //don't visualize, if nobody's looking

        visualization_msgs::Marker marker_lines;

        marker_lines.header.frame_id = "/openni_rgb_optical_frame";
        marker_lines.ns = "ransac_markers";
        marker_lines.header.stamp = ros::Time::now();
        marker_lines.action = visualization_msgs::Marker::ADD;
        marker_lines.pose.orientation.w = 1.0;
        marker_lines.id = marker_id;
        marker_lines.type = visualization_msgs::Marker::LINE_LIST;
        marker_lines.scale.x = 0.002;
        
        std_msgs::ColorRGBA color_red  ;  //red outlier
        color_red.r = 1.0;
        color_red.a = 1.0;
        std_msgs::ColorRGBA color_green;  //green inlier, newer endpoint
        color_green.g = 1.0;
        color_green.a = 1.0;
        std_msgs::ColorRGBA color_yellow;  //yellow inlier, earlier endpoint
        color_yellow.r = 1.0;
        color_yellow.g = 1.0;
        color_yellow.a = 1.0;
        std_msgs::ColorRGBA color_blue  ;  //red-blue outlier
        color_blue.b = 1.0;
        color_blue.a = 1.0;

        marker_lines.color = color_green; //just to set the alpha channel to non-zero
        const g2o::VertexSE3* earlier_v; //used to get the transform
        const g2o::VertexSE3* newer_v; //used to get the transform
        VertexIDMap v_idmap = optimizer_->vertices();
        // end of initialization
        ROS_DEBUG("Matches Visualization start: %lu Matches, %lu Inliers", last_matches_.size(), last_inlier_matches_.size());

        // write all inital matches to the line_list
        marker_lines.points.clear();//necessary?

        if (draw_outlier)
        {
            for (unsigned int i=0;i<last_matches_.size(); i++){
                int newer_id = last_matches_.at(i).queryIdx; //feature id in newer node
                int earlier_id = last_matches_.at(i).trainIdx; //feature id in earlier node

                earlier_v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(last_matching_node_));
                newer_v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_.size()-1));

                //Outliers are red (newer) to blue (older)
                marker_lines.colors.push_back(color_red);
                marker_lines.colors.push_back(color_blue);

                Node* last = graph_.find(graph_.size()-1)->second;
                marker_lines.points.push_back(
                        pointInWorldFrame(last->feature_locations_3d_[newer_id], newer_v->estimate()));
                Node* prev = graph_.find(last_matching_node_)->second;
                marker_lines.points.push_back(
                        pointInWorldFrame(prev->feature_locations_3d_[earlier_id], earlier_v->estimate()));
            }
        }

        for (unsigned int i=0;i<last_inlier_matches_.size(); i++){
            int newer_id = last_inlier_matches_.at(i).queryIdx; //feature id in newer node
            int earlier_id = last_inlier_matches_.at(i).trainIdx; //feature id in earlier node

            earlier_v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(last_matching_node_));
            newer_v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_.size()-1));


            //inliers are green (newer) to blue (older)
            marker_lines.colors.push_back(color_green);
            marker_lines.colors.push_back(color_blue);

            Node* last = graph_.find(graph_.size()-1)->second;
            marker_lines.points.push_back(
                    pointInWorldFrame(last->feature_locations_3d_[newer_id], newer_v->estimate()));
            Node* prev = graph_.find(last_matching_node_)->second;
            marker_lines.points.push_back(
                    pointInWorldFrame(prev->feature_locations_3d_[earlier_id], earlier_v->estimate()));
        }

        ransac_marker_pub_.publish(marker_lines);
        ROS_DEBUG_STREAM("Published  " << marker_lines.points.size()/2 << " lines");
    }
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}


void GraphManager::visualizeGraphEdges() const {
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);

    if (marker_pub_.getNumSubscribers() > 0){ //no visualization for nobody
        visualization_msgs::Marker edges_marker;
        edges_marker.header.frame_id = "/openni_rgb_optical_frame"; //TODO: Should be a meaningfull fixed frame with known relative pose to the camera
        edges_marker.header.stamp = ros::Time::now();
        edges_marker.ns = "camera_pose_graph"; // Set the namespace and id for this marker.  This serves to create a unique ID
        edges_marker.id = 0;    // Any marker sent with the same namespace and id will overwrite the old one

        edges_marker.type = visualization_msgs::Marker::LINE_LIST;
        edges_marker.action = visualization_msgs::Marker::ADD; // Set the marker action.  Options are ADD and DELETE
        edges_marker.frame_locked = true; //rviz automatically retransforms the markers into the frame every update cycle
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        edges_marker.scale.x = 0.005; //line width
        //Global pose (used to transform all points)
        edges_marker.pose.position.x = 0;
        edges_marker.pose.position.y = 0;
        edges_marker.pose.position.z = 0;
        edges_marker.pose.orientation.x = 0.0;
        edges_marker.pose.orientation.y = 0.0;
        edges_marker.pose.orientation.z = 0.0;
        edges_marker.pose.orientation.w = 1.0;
        // Set the color -- be sure to set alpha to something non-zero!
        edges_marker.color.r = 1.0f;
        edges_marker.color.g = 1.0f;
        edges_marker.color.b = 1.0f;
        edges_marker.color.a = 0.5f;//looks smoother
        geometry_msgs::Point point; //start and endpoint for each line segment
        g2o::VertexSE3* v1,* v2; //used in loop
        EdgeSet::iterator edge_iter = optimizer_->edges().begin();
        int counter = 0;
        for(;edge_iter != optimizer_->edges().end(); edge_iter++, counter++) {
            g2o::EdgeSE3* myedge = dynamic_cast<g2o::EdgeSE3*>(*edge_iter);
            std::vector<g2o::HyperGraph::Vertex*>& myvertices = myedge->vertices();
            v1 = dynamic_cast<g2o::VertexSE3*>(myvertices.at(1));
            v2 = dynamic_cast<g2o::VertexSE3*>(myvertices.at(0));

            point.x = v1->estimate().translation().x();
            point.y = v1->estimate().translation().y();
            point.z = v1->estimate().translation().z();
            edges_marker.points.push_back(point);
            
            point.x = v2->estimate().translation().x();
            point.y = v2->estimate().translation().y();
            point.z = v2->estimate().translation().z();
            edges_marker.points.push_back(point);
        }

        marker_pub_.publish (edges_marker);
        ROS_INFO("published %d graph edges", counter);
    }

    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}

void GraphManager::visualizeGraphNodes() const {
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);

    if (marker_pub_.getNumSubscribers() > 0){ //don't visualize, if nobody's looking
        visualization_msgs::Marker nodes_marker;
        nodes_marker.header.frame_id = "/openni_rgb_optical_frame"; //TODO: Should be a meaningfull fixed frame with known relative pose to the camera
        nodes_marker.header.stamp = ros::Time::now();
        nodes_marker.ns = "camera_pose_graph"; // Set the namespace and id for this marker.  This serves to create a unique ID
        nodes_marker.id = 1;    // Any marker sent with the same namespace and id will overwrite the old one


        nodes_marker.type = visualization_msgs::Marker::LINE_LIST;
        nodes_marker.action = visualization_msgs::Marker::ADD; // Set the marker action.  Options are ADD and DELETE
        nodes_marker.frame_locked = true; //rviz automatically retransforms the markers into the frame every update cycle
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        nodes_marker.scale.x = 0.002;
        //Global pose (used to transform all points) //TODO: is this the default pose anyway?
        nodes_marker.pose.position.x = 0;
        nodes_marker.pose.position.y = 0;
        nodes_marker.pose.position.z = 0;
        nodes_marker.pose.orientation.x = 0.0;
        nodes_marker.pose.orientation.y = 0.0;
        nodes_marker.pose.orientation.z = 0.0;
        nodes_marker.pose.orientation.w = 1.0;
        // Set the color -- be sure to set alpha to something non-zero!
        nodes_marker.color.r = 1.0f;
        nodes_marker.color.g = 0.0f;
        nodes_marker.color.b = 0.0f;
        nodes_marker.color.a = 1.0f;


        geometry_msgs::Point tail; //same startpoint for each line segment
        geometry_msgs::Point tip;  //different endpoint for each line segment
        std_msgs::ColorRGBA arrow_color_red  ;  //red x axis
        arrow_color_red.r = 1.0;
        arrow_color_red.a = 1.0;
        std_msgs::ColorRGBA arrow_color_green;  //green y axis
        arrow_color_green.g = 1.0;
        arrow_color_green.a = 1.0;
        std_msgs::ColorRGBA arrow_color_blue ;  //blue z axis
        arrow_color_blue.b = 1.0;
        arrow_color_blue.a = 1.0;
        Eigen::Vector3d origin(0.0,0.0,0.0);
        Eigen::Vector3d x_axis(0.2,0.0,0.0); //20cm long axis for the first (almost fixed) node
        Eigen::Vector3d y_axis(0.0,0.2,0.0);
        Eigen::Vector3d z_axis(0.0,0.0,0.2);
        Eigen::Vector3d tmp; //the transformed endpoints
        int counter = 0;
        g2o::VertexSE3* v; //used in loop
        VertexIDMap::iterator vertex_iter = optimizer_->vertices().begin();
        for(/*see above*/; vertex_iter != optimizer_->vertices().end(); vertex_iter++, counter++) {
            v = dynamic_cast<g2o::VertexSE3* >((*vertex_iter).second);
            //v->estimate().rotation().x()+ v->estimate().rotation().y()+ v->estimate().rotation().z()+ v->estimate().rotation().w();
            tmp = v->estimate() * origin;
            tail.x = tmp.x();
            tail.y = tmp.y();
            tail.z = tmp.z();
            //Endpoints X-Axis
            nodes_marker.points.push_back(tail);
            nodes_marker.colors.push_back(arrow_color_red);
            tmp = v->estimate() * x_axis;
            tip.x  = tmp.x();
            tip.y  = tmp.y();
            tip.z  = tmp.z();
            nodes_marker.points.push_back(tip);
            nodes_marker.colors.push_back(arrow_color_red);
            //Endpoints Y-Axis
            nodes_marker.points.push_back(tail);
            nodes_marker.colors.push_back(arrow_color_green);
            tmp = v->estimate() * y_axis;
            tip.x  = tmp.x();
            tip.y  = tmp.y();
            tip.z  = tmp.z();
            nodes_marker.points.push_back(tip);
            nodes_marker.colors.push_back(arrow_color_green);
            //Endpoints Z-Axis
            nodes_marker.points.push_back(tail);
            nodes_marker.colors.push_back(arrow_color_blue);
            tmp = v->estimate() * z_axis;
            tip.x  = tmp.x();
            tip.y  = tmp.y();
            tip.z  = tmp.z();
            nodes_marker.points.push_back(tip);
            nodes_marker.colors.push_back(arrow_color_blue);
            //shorten all nodes after the first one
            x_axis.x() = 0.1;
            y_axis.y() = 0.1;
            z_axis.z() = 0.1;
        }

        marker_pub_.publish (nodes_marker);
        ROS_INFO("published %d graph nodes", counter);
    }

    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}

bool GraphManager::addEdgeToG2O(const LoadedEdge3D& edge, bool largeEdge, bool set_estimate) {
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);

    QMutexLocker locker(&optimizer_mutex);
    g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(edge.id1));
    g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(edge.id2));

    // at least one vertex has to be created, assert that the transformation
    // is large enough to avoid to many vertices on the same spot
    if (!v1 || !v2){
        if (!largeEdge) {
            ROS_INFO("Edge to new vertex is to short, vertex will not be inserted");
            return false; 
        }
    }

    if(!v1 && !v2){
      ROS_ERROR("Missing both vertices: %i, %i, cannot create edge", edge.id1, edge.id2);
      return false;
    }
    else if (!v1 && v2) {
        v1 = new g2o::VertexSE3;
        assert(v1);
        v1->setId(edge.id1);
        v1->setEstimate(v2->estimate() * edge.mean.inverse());
        optimizer_->addVertex(v1); 
        latest_transform_ = g2o2QMatrix(v1->estimate()); 
    }
    else if (!v2 && v1) {
        v2 = new g2o::VertexSE3;
        assert(v2);
        v2->setId(edge.id2);
        v2->setEstimate(v1->estimate() * edge.mean);
        optimizer_->addVertex(v2); 
        latest_transform_ = g2o2QMatrix(v2->estimate()); 
    }
    else if(set_estimate){
        v2->setEstimate(v1->estimate() * edge.mean);
    }
    g2o::EdgeSE3* g2o_edge = new g2o::EdgeSE3;
    g2o_edge->vertices()[0] = v1;
    g2o_edge->vertices()[1] = v2;
    g2o_edge->setMeasurement(edge.mean);
    g2o_edge->setInverseMeasurement(edge.mean.inverse());
    g2o_edge->setInformation(edge.informationMatrix);
    optimizer_->addEdge(g2o_edge);

    if(abs(edge.id1 - edge.id2) > ParameterServer::instance()->get<int>("predecessor_candidates")){
      loop_closures_edges++;
    } else {
      sequential_edges++;
    }

    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
    return true;
}

void GraphManager::optimizeGraph(int max_iter, bool nonthreaded){
  if(ParameterServer::instance()->get<bool>("concurrent_optimization") && !nonthreaded) {
    ROS_DEBUG("Optimization done in Thread");
    QtConcurrent::run(this, &GraphManager::optimizeGraphImpl, max_iter); 
  }
  else { //Non-concurrent
    optimizeGraphImpl(max_iter);//regular function call
  }
}

void GraphManager::optimizeGraphImpl(int max_iter)
{
  struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
  int iterations = max_iter >= 0 ? max_iter : ParameterServer::instance()->get<int>("optimizer_iterations");
  QMutexLocker locker(&optimizer_mutex);

  ROS_WARN_NAMED("eval", "Loop Closures: %u, Sequential Edges: %u", loop_closures_edges, sequential_edges);
  if(ParameterServer::instance()->get<std::string>("backend_solver") == "auto"){
    if(static_cast<float>(sequential_edges)/static_cast<float>(loop_closures_edges) < 1.0){
      ROS_WARN_NAMED("eval", "Using PCG as solver");
      if(current_backend_ != "pcg") createOptimizer("pcg", optimizer_);
    } else {
      ROS_WARN_NAMED("eval", "Using CHOLMOD as solver");
      if(current_backend_ != "cholmod") createOptimizer("cholmod", optimizer_);
    }
  }

  ROS_WARN("Starting Optimization");
  std::string bagfile_name = ParameterServer::instance()->get<std::string>("bagfile_name");
  optimizer_->save((bagfile_name + "_g2o-optimizer-save-file-before").c_str());
  optimizer_->initializeOptimization();
  double prev_chi2 = std::numeric_limits<double>::max();
  int i = 0;
  for(; i <  iterations; i++){
    int currentIt = optimizer_->optimize(1);
    optimizer_->computeActiveErrors();
    ROS_WARN_STREAM_NAMED("eval", "G2O Statistics: " << std::setprecision(15) << optimizer_->vertices().size() << " nodes, " 
                    << optimizer_->edges().size() << " edges. "
                    << optimizer_->chi2() << " ; chi2 "<< ", Iterations: " << currentIt);
    if(!(prev_chi2 - optimizer_->chi2() > optimizer_->chi2()*0.0001)){ // improvement less than 0.01 Percent
      ROS_INFO("G2O Converged after %d iterations", i);
      break;
    }
    prev_chi2 = optimizer_->chi2();
  }
  ROS_INFO("Finished G2O optimization after %d iterations", i);
  optimizer_->save((bagfile_name + "_g2o-optimizer-save-file-after").c_str());

  g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(optimizer_->vertices().size()-1));

  computed_motion_ =  g2o2TF(v->estimate());
  latest_transform_ = g2o2QMatrix(v->estimate()); 
  Q_EMIT setGraphEdges(getGraphEdges());
  Q_EMIT updateTransforms(getAllPosesAsMatrixList());

  clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
  ROS_WARN_STREAM_NAMED("eval", "Optimizer Runtime; "<< elapsed <<" s");
}


/**
 * Publish the updated transforms for the graph node resp. clouds
 *
void GraphManager::publishCorrectedTransforms(){
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
    //fill message
    rgbdslam::CloudTransforms msg;
    for (unsigned int i = 0; i < optimizer_->vertices().size(); ++i) {
        g2o::VertexSE3* v = optimizer_->vertex(i);
        tf::Transform trans = g2o2TF(v->estimate());
        geometry_msgs::Transform trans_msg;
        tf::transformTFToMsg(trans,trans_msg);
        msg.transforms.push_back(trans_msg);
        msg.ids.push_back(graph_[i]->msg_id_); //msg_id is no more
    }
    msg.header.stamp = ros::Time::now();

    if (transform_pub_.getNumSubscribers() > 0)
        transform_pub_.publish(msg);
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}*/

void GraphManager::reset(){
    reset_request_ = true;
}

void GraphManager::deleteLastFrame(){
    if(graph_.size() <= 1) {
      ROS_INFO("Resetting, as the only node is to be deleted");
      reset_request_ = true;
      Q_EMIT deleteLastNode();
      return;
    }
    optimizer_mutex.lock();
    g2o::VertexSE3* v_to_del = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(optimizer_->vertices().size()-1));//last vertex
    g2o::VertexSE3 *v1, *v2; //used in loop as temporaries
    EdgeSet::iterator edge_iter = optimizer_->edges().begin();
    for(;edge_iter != optimizer_->edges().end(); edge_iter++) {
        g2o::EdgeSE3* myedge = dynamic_cast<g2o::EdgeSE3*>(*edge_iter);
        std::vector<g2o::HyperGraph::Vertex*>& myvertices = myedge->vertices();
        v1 = dynamic_cast<g2o::VertexSE3*>(myvertices.at(1));
        v2 = dynamic_cast<g2o::VertexSE3*>(myvertices.at(0));
        if(v1->id() == v_to_del->id() || v2->id() == v_to_del->id()) 
          optimizer_->removeEdge((*edge_iter));
    }

    optimizer_->removeVertex(v_to_del);
    graph_.erase(graph_.size()-1);
    optimizer_mutex.unlock();
    Q_EMIT deleteLastNode();
    optimizeGraph();//s.t. the effect of the removed edge transforms are removed to
    ROS_INFO("Removed most recent node");
    Q_EMIT setGUIInfo("Removed most recent node");
    //Q_EMIT setGraphEdges(getGraphEdges());
    //updateTransforms needs to be last, as it triggers a redraw
    //Q_EMIT updateTransforms(getAllPosesAsMatrixList());
}

QList<QPair<int, int> >* GraphManager::getGraphEdges()
{
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
    //QList<QPair<int, int> >* edge_list = new QList<QPair<int, int> >();
    current_edges_.clear();
    g2o::VertexSE3 *v1, *v2; //used in loop
    EdgeSet::iterator edge_iter = optimizer_->edges().begin();
    for(;edge_iter != optimizer_->edges().end(); edge_iter++) {
        g2o::EdgeSE3* myedge = dynamic_cast<g2o::EdgeSE3*>(*edge_iter);
        std::vector<g2o::HyperGraph::Vertex*>& myvertices = myedge->vertices();
        v1 = dynamic_cast<g2o::VertexSE3*>(myvertices.at(1));
        v2 = dynamic_cast<g2o::VertexSE3*>(myvertices.at(0));
        current_edges_.append( qMakePair(v1->id(), v2->id()));
    }
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
    return new QList<QPair<int, int> >(current_edges_);
}

QList<QMatrix4x4>* GraphManager::getAllPosesAsMatrixList(){
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
    ROS_DEBUG("Retrieving all transformations from optimizer");
    //QList<QMatrix4x4>* result = new QList<QMatrix4x4>();
    current_poses_.clear();
#if defined(QT_VERSION) && QT_VERSION >= 0x040700
    current_poses_.reserve(optimizer_->vertices().size());//only allocates the internal pointer array
#endif

    for (unsigned int i = 0; i < optimizer_->vertices().size(); ++i) {
        g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(i));
        if(v){ 
            current_poses_.push_back(g2o2QMatrix(v->estimate())); 
        } else {
            ROS_ERROR("Nullpointer in graph at position %i!", i);
        }
    }
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
    return new QList<QMatrix4x4>(current_poses_); //pointer to a copy
}
// If QT Concurrent is available, run the saving in a seperate thread
void GraphManager::saveAllClouds(QString filename, bool threaded){
    if (ParameterServer::instance()->get<bool>("concurrent_edge_construction") && threaded) {
        QFuture<void> f1 = QtConcurrent::run(this, &GraphManager::saveAllCloudsToFile, filename);
        //f1.waitForFinished();
    }
    else {// Otherwise just call it without threading
        saveAllCloudsToFile(filename);
    }
}

void GraphManager::saveIndividualClouds(QString filename, bool threaded){
  if (ParameterServer::instance()->get<bool>("concurrent_edge_construction") && threaded) {
    QFuture<void> f1 = QtConcurrent::run(this, &GraphManager::saveIndividualCloudsToFile, filename);
    //f1.waitForFinished();
  }
  else {
    saveIndividualCloudsToFile(filename);
  }
}

void GraphManager::saveIndividualCloudsToFile(QString file_basename)
{
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
    ROS_INFO("Saving all clouds to %sxxxx.pcd", qPrintable(file_basename));
    std::string gt = ParameterServer::instance()->get<std::string>("ground_truth_frame_name");
    ROS_INFO_COND(!gt.empty(), "Saving all clouds with ground truth sensor position to gt_%sxxxx.pcd", qPrintable(file_basename));

    batch_processing_runs_ = true;
    tf::Transform  world2base;
    QString message, filename;
    std::string fixed_frame_id = ParameterServer::instance()->get<std::string>("fixed_frame_name");
    for (unsigned int i = 0; i < optimizer_->vertices().size(); ++i) {
        g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(i));
        if(!v){ 
            ROS_ERROR("Nullpointer in graph at position %i!", i);
            continue;
        }
        if(graph_[i]->pc_col->size() == 0){
            ROS_INFO("Skipping Node %i, point cloud data is empty!", i);
            continue;
        }
        /*/TODO: is all this correct?
        tf::Transform transform = g2o2TF(v->estimate());
        tf::Transform cam2rgb;
        cam2rgb.setRotation(tf::createQuaternionFromRPY(-1.57,0,-1.57));
        cam2rgb.setOrigin(tf::Point(0,-0.04,0));
        world2base = cam2rgb*transform;
        */
        tf::Transform pose = g2o2TF(v->estimate());
        tf::StampedTransform base2points = graph_[i]->getBase2PointsTransform();//get pose of base w.r.t current pc at capture time
        world2base = init_base_pose_*base2points*pose*base2points.inverse();

        Eigen::Vector4f sensor_origin(world2base.getOrigin().x(),world2base.getOrigin().y(),world2base.getOrigin().z(),world2base.getOrigin().w());
        Eigen::Quaternionf sensor_orientation(world2base.getRotation().w(),world2base.getRotation().x(),world2base.getRotation().y(),world2base.getRotation().z());

        graph_[i]->pc_col->sensor_origin_ = sensor_origin;
        graph_[i]->pc_col->sensor_orientation_ = sensor_orientation;
        graph_[i]->pc_col->header.frame_id = fixed_frame_id;

        filename.sprintf("%s_%04d.pcd", qPrintable(file_basename), i);
        Q_EMIT setGUIStatus(message.sprintf("Saving to %s: Transformed Node %i/%i", qPrintable(filename), i, (int)optimizer_->vertices().size()));
        pcl::io::savePCDFile(qPrintable(filename), *(graph_[i]->pc_col), true); //Last arg: true is binary mode. ASCII mode drops color bits
        
        if(!gt.empty()){
          tf::StampedTransform gt_world2base = graph_[i  ]->getGroundTruthTransform();//get mocap pose of base in map
          if( gt_world2base.frame_id_   == "/missing_ground_truth" ){ 
            ROS_WARN_STREAM("Skipping ground truth: " << gt_world2base.child_frame_id_ << " child/parent " << gt_world2base.frame_id_);
            continue;
          }
          Eigen::Vector4f sensor_origin(gt_world2base.getOrigin().x(),gt_world2base.getOrigin().y(),gt_world2base.getOrigin().z(),gt_world2base.getOrigin().w());
          Eigen::Quaternionf sensor_orientation(gt_world2base.getRotation().w(),gt_world2base.getRotation().x(),gt_world2base.getRotation().y(),gt_world2base.getRotation().z());

          graph_[i]->pc_col->sensor_origin_ = sensor_origin;
          graph_[i]->pc_col->sensor_orientation_ = sensor_orientation;
          graph_[i]->pc_col->header.frame_id = fixed_frame_id;

          filename.sprintf("%s_%04d_gt.pcd", qPrintable(file_basename), i);
          Q_EMIT setGUIStatus(message.sprintf("Saving to %s: Transformed Node %i/%i", qPrintable(filename), i, (int)optimizer_->vertices().size()));
          pcl::io::savePCDFile(qPrintable(filename), *(graph_[i]->pc_col), true); //Last arg: true is binary mode. ASCII mode drops color bits
        }
  
    }
    Q_EMIT setGUIStatus("Saved all point clouds");
    ROS_INFO ("Saved all points clouds to %sxxxx.pcd", qPrintable(file_basename));
    batch_processing_runs_ = false;
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}

void GraphManager::saveAllFeatures(QString filename, bool threaded)
{
    if (ParameterServer::instance()->get<bool>("concurrent_edge_construction") && threaded) {
        QFuture<void> f1 = QtConcurrent::run(this, &GraphManager::saveAllFeaturesToFile, filename);
        //f1.waitForFinished();
    }
    else {// Otherwise just call it without threading
        saveAllFeaturesToFile(filename);
    }
}
void GraphManager::saveAllFeaturesToFile(QString filename)
{
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
    cv::FileStorage fs(qPrintable(filename), cv::FileStorage::WRITE);
    fs << "Feature_Locations" << "[";

    ROS_INFO("Saving all features to %s transformed to a common coordinate frame.", qPrintable(filename));
    batch_processing_runs_ = true;
    tf::Transform  world2rgb, cam2rgb;
    QString message;
    cam2rgb.setRotation(tf::createQuaternionFromRPY(-1.57,0,-1.57));
    cam2rgb.setOrigin(tf::Point(0,-0.04,0));
    int feat_count = 0;
    for (unsigned int i = 0; i < optimizer_->vertices().size(); ++i) {
        g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(i));
        tf::Transform world2cam = g2o2TF(v->estimate());
        world2rgb = cam2rgb*world2cam;
        Eigen::Matrix4f world2rgbMat;
        pcl_ros::transformAsMatrix(world2rgb, world2rgbMat);
        BOOST_FOREACH(Eigen::Vector4f loc, graph_[i]->feature_locations_3d_)
        {
          loc.w() = 1.0;
          Eigen::Vector4f new_loc = world2rgbMat * loc;
          fs << "{:" << "x" << new_loc.x() << "y" << new_loc.y() << "z" << new_loc.z() << "}";
          feat_count++;
        }
        Q_EMIT setGUIStatus(message.sprintf("Saving to %s: Features of Node %i/%i", qPrintable(filename), i, (int)optimizer_->vertices().size()));
    }
    fs << "]";
    //Assemble all descriptors into a big matrix
    int descriptor_size = graph_[0]->feature_descriptors_.cols;
    int descriptor_type = graph_[0]->feature_descriptors_.type();
    cv::Mat alldescriptors(0, descriptor_size,  descriptor_type);
    alldescriptors.reserve(feat_count);
    for (unsigned int i = 0; i < graph_.size(); ++i) {
      alldescriptors.push_back(graph_[i]->feature_descriptors_);
    }
    fs << "Feature_Descriptors" << alldescriptors;
    fs.release();

    Q_EMIT setGUIStatus(message.sprintf("Saved %d features points to %s", feat_count, qPrintable(filename)));
    ROS_INFO ("Saved %d feature points to %s", feat_count, qPrintable(filename));
    batch_processing_runs_ = false;
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}




void GraphManager::saveAllCloudsToFile(QString filename){
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
    pointcloud_type aggregate_cloud; ///will hold all other clouds
    ROS_INFO("Saving all clouds to %s, this may take a while as they need to be transformed to a common coordinate frame.", qPrintable(filename));
    batch_processing_runs_ = true;
    tf::Transform  world2cam;
    //fill message
    //rgbdslam::CloudTransforms msg;
    QString message;
    tf::Transform cam2rgb;
    cam2rgb.setRotation(tf::createQuaternionFromRPY(-1.57,0,-1.57));
    cam2rgb.setOrigin(tf::Point(0,-0.04,0));
    for (unsigned int i = 0; i < optimizer_->vertices().size(); ++i) {
        g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(i));
        if(!v){ 
            ROS_ERROR("Nullpointer in graph at position %i!", i);
            continue;
        }
        tf::Transform transform = g2o2TF(v->estimate());
        world2cam = cam2rgb*transform;
        transformAndAppendPointCloud (*(graph_[i]->pc_col), aggregate_cloud, world2cam, Max_Depth);

        if(ParameterServer::instance()->get<bool>("batch_processing"))
          graph_[i]->clearPointCloud(); //saving all is the last thing to do, so these are not required anymore
        Q_EMIT setGUIStatus(message.sprintf("Saving to %s: Transformed Node %i/%i", qPrintable(filename), i, (int)optimizer_->vertices().size()));
    }
    aggregate_cloud.header.frame_id = "/openni_camera";
    if(filename.endsWith(".ply", Qt::CaseInsensitive))
      pointCloud2MeshFile(filename, aggregate_cloud);
    if(filename.endsWith(".pcd", Qt::CaseInsensitive))
      pcl::io::savePCDFile(qPrintable(filename), aggregate_cloud, true); //Last arg is binary mode
    else {
      ROS_WARN("Filename misses correct extension (.pcd or .ply) using .pcd");
      filename.append(".pcd");
      pcl::io::savePCDFile(qPrintable(filename), aggregate_cloud, true); //Last arg is binary mode
    }
    Q_EMIT setGUIStatus(message.sprintf("Saved %d data points to %s", (int)aggregate_cloud.points.size(), qPrintable(filename)));
    ROS_INFO ("Saved %d data points to %s", (int)aggregate_cloud.points.size(), qPrintable(filename));

    if (whole_cloud_pub_.getNumSubscribers() > 0){ //if it should also be send out
        sensor_msgs::PointCloud2 cloudMessage_; //this will be send out in batch mode
        pcl::toROSMsg(aggregate_cloud,cloudMessage_);
        cloudMessage_.header.frame_id = "/openni_camera";
        cloudMessage_.header.stamp = ros::Time::now();
        whole_cloud_pub_.publish(cloudMessage_);
        ROS_INFO("Aggregate pointcloud sent");
    }
    batch_processing_runs_ = false;
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}

void GraphManager::pointCloud2MeshFile(QString filename, pointcloud_type full_cloud){
  QFile file(filename);//file is closed on destruction
  if(!file.open(QIODevice::WriteOnly|QIODevice::Text)){
    ROS_ERROR("Could not open file %s", qPrintable(filename));
    return; 
  }
  QTextStream out(&file);
	out << "ply\n";
	out << "format ascii 1.0\n";
	out << "element vertex " << (int)full_cloud.points.size() << "\n"; 
	out << "property float x\n";
	out << "property float y\n";
	out << "property float z\n";
	out << "property uchar red\n";
	out << "property uchar green\n";
	out << "property uchar blue\n";
	out << "end_header\n";
  unsigned char r,g,b;
  float x, y, z ;
  for(unsigned int i = 0; i < full_cloud.points.size() ; i++){
    b = *(  (unsigned char*)&(full_cloud.points[i].rgb));
    g = *(1+(unsigned char*)&(full_cloud.points[i].rgb));
    r = *(2+(unsigned char*)&(full_cloud.points[i].rgb));
    x = full_cloud.points[i].x;
    y = full_cloud.points[i].y;
    z = full_cloud.points[i].z;
    out << qSetFieldWidth(8) << x << " " << y << " " << z << " ";
    out << qSetFieldWidth(3) << r << " " << g << " " << b << "\n";
  }
}
  

void GraphManager::saveTrajectory(QString filebasename, bool with_ground_truth){
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
    if(graph_.size() == 0){
      ROS_ERROR("Graph is empty, no trajectory can be saved");
      return;
    }
    ROS_INFO("Logging Trajectory");
    QMutexLocker locker(&optimizer_mutex);

    //FIXME: DO this block only if with_ground_truth is true and !gt.empty()
    std::string gt = ParameterServer::instance()->get<std::string>("ground_truth_frame_name");

    ROS_INFO("Comparison of relative motion with ground truth");
    QString gtt_fname("_ground_truth.txt");
    QFile gtt_file(gtt_fname.prepend(filebasename));//file is closed on destruction
    if(!gtt_file.open(QIODevice::WriteOnly|QIODevice::Text)) return; //TODO: Errormessage
    QTextStream gtt_out(&gtt_file);
    tf::StampedTransform b2p = graph_[0]->getGroundTruthTransform();
    gtt_out.setRealNumberNotation(QTextStream::FixedNotation);
    gtt_out << "# TF Coordinate Frame ID: " << b2p.frame_id_.c_str() << "(data: " << b2p.child_frame_id_.c_str() << ")\n";

     
    QString et_fname("_estimate.txt");
    QFile et_file (et_fname.prepend(filebasename));//file is closed on destruction
    if(!et_file.open(QIODevice::WriteOnly|QIODevice::Text)) return; //TODO: Errormessage
    QTextStream et_out(&et_file);
    et_out.setRealNumberNotation(QTextStream::FixedNotation);
    b2p = graph_[0]->getBase2PointsTransform();
    et_out << "# TF Coordinate Frame ID: " << b2p.frame_id_.c_str() << "(data: " << b2p.child_frame_id_.c_str() << ")\n";

    for (unsigned int i = 0; i < optimizer_->vertices().size(); ++i) {
        g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(i));
        ROS_ERROR_COND(!v, "Nullpointer in graph at position %i!", i);

        tf::Transform pose = g2o2TF(v->estimate());

        tf::StampedTransform base2points = graph_[i]->getBase2PointsTransform();//get pose of base w.r.t current pc at capture time
        tf::Transform world2base = init_base_pose_*base2points*pose*base2points.inverse();

        logTransform(et_out, world2base, graph_[i]->pc_col->header.stamp.toSec()); 
        //Eigen::Matrix<double, 6,6> uncertainty = v->uncertainty();
        //et_out << uncertainty(0,0) << "\t" << uncertainty(1,1) << "\t" << uncertainty(2,2) << "\t" << uncertainty(3,3) << "\t" << uncertainty(4,4) << "\t" << uncertainty(5,5) <<"\n" ;
        if(with_ground_truth && !gt.empty()){
          tf::StampedTransform gt_world2base = graph_[i  ]->getGroundTruthTransform();//get mocap pose of base in map
          if( gt_world2base.frame_id_   == "/missing_ground_truth" ){ 
            ROS_WARN_STREAM("Skipping ground truth: " << gt_world2base.child_frame_id_ << " child/parent " << gt_world2base.frame_id_);
            continue;
          }
          logTransform(gtt_out, gt_world2base, gt_world2base.stamp_.toSec()); 
          //logTransform(et_out, world2base, gt_world2base.stamp_.toSec()); 
        } 
    }
    ROS_INFO_COND(!gt.empty() && with_ground_truth, "Written logfiles ground_truth_trajectory.txt and estimated_trajectory.txt");
    ROS_INFO_COND(gt.empty(),  "Written logfile estimated_trajectory.txt");
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}

void GraphManager::broadcastTransform(Node* node, tf::Transform& computed_motion){
    std::string fixed_frame = ParameterServer::instance()->get<std::string>("fixed_frame_name");
    std::string base_frame  = ParameterServer::instance()->get<std::string>("base_frame_name");
    
    /*
    if(graph_.size() == 0){
      ROS_WARN("Cannot broadcast transform while graph is empty sending identity");
      br_.sendTransform(tf::StampedTransform(tf::Transform::getIdentity(), ros::Time::now(), fixed_frame, base_frame));
      return;
    }
    */
    const tf::StampedTransform& base2points = node->getBase2PointsTransform();//get pose of base w.r.t current pc at capture time

    //Assumption: computed_motion_ contains last pose
    tf::Transform world2base = init_base_pose_*base2points*computed_motion*base2points.inverse();
    printTransform("World->Base", world2base);

    ROS_DEBUG("Broadcasting transform");
    
    br_.sendTransform(tf::StampedTransform(world2base.inverse(), base2points.stamp_, base_frame, fixed_frame));
}

// If QT Concurrent is available, run the saving in a seperate thread
void GraphManager::sendAllClouds(bool threaded){
    if (ParameterServer::instance()->get<bool>("concurrent_edge_construction") && threaded) {
        QFuture<void> f1 = QtConcurrent::run(this, &GraphManager::sendAllCloudsImpl);
        //f1.waitForFinished();
    }
    else {// Otherwise just call it without threading
        sendAllCloudsImpl();
    }
}


void GraphManager::sendAllCloudsImpl(){
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
    if (batch_cloud_pub_.getNumSubscribers() == 0){
        ROS_WARN("No Subscribers: Sending of clouds cancelled");
        return;
    }

    ROS_INFO("Sending out all clouds");
    batch_processing_runs_ = true;
    ros::Rate r(ParameterServer::instance()->get<double>("send_clouds_rate")); //slow down a bit, to allow for transmitting to and processing in other nodes

    for (unsigned int i = 0; i < optimizer_->vertices().size(); ++i) {
        g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(i));
        if(!v){ 
            ROS_ERROR("Nullpointer in graph at position %i!", i);
            continue;
        }

        tf::Transform base2points = graph_[i]->getBase2PointsTransform();//get pose of base w.r.t current pc at capture time
        printTransform("base2points", base2points);
        tf::Transform computed_motion = g2o2TF(v->estimate());//get pose of point cloud w.r.t. first frame's pc
        printTransform("computed_motion", computed_motion);
        printTransform("init_base_pose_", init_base_pose_);

        tf::Transform world2base = init_base_pose_*base2points*computed_motion*base2points.inverse();
        tf::Transform gt_world2base = graph_[i]->getGroundTruthTransform();//get mocap pose of base in map
        tf::Transform err = gt_world2base.inverseTimes(world2base);
        //TODO: Compute err from relative transformations betw. time steps

        ros::Time now = ros::Time::now(); //makes sure things have a corresponding timestamp
        ROS_DEBUG("Sending out transform %i", i);
        printTransform("World->Base", world2base);
        std::string fixed_frame = ParameterServer::instance()->get<std::string>("fixed_frame_name");
        br_.sendTransform(tf::StampedTransform(world2base, now, fixed_frame, "/openni_camera"));
        br_.sendTransform(tf::StampedTransform(err, now, fixed_frame, "/where_mocap_should_be"));
        ROS_DEBUG("Sending out cloud %i", i);
        //graph_[i]->publish("/batch_transform", now, batch_cloud_pub_);
        graph_[i]->publish("/openni_rgb_optical_frame", now, batch_cloud_pub_);
        //tf::Transform ground_truth_tf = graph_[i]->getGroundTruthTransform();
        QString message;
        Q_EMIT setGUIInfo(message.sprintf("Sending pointcloud and map transform (%i/%i) on topics %s and /tf", (int)i+1, (int)optimizer_->vertices().size(), ParameterServer::instance()->get<std::string>("individual_cloud_out_topic").c_str()) );
        r.sleep();
    }

    batch_processing_runs_ = false;
    Q_EMIT sendFinished();
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}

void GraphManager::setMaxDepth(float max_depth){
	Max_Depth = max_depth;
	ROS_INFO("Max Depth set to: %f", max_depth);
}

void GraphManager::cloudRendered(pointcloud_type const * pc) {
  BOOST_REVERSE_FOREACH(GraphNodeType entry, graph_){
    if(entry.second->pc_col.get() == pc){
      entry.second->clearPointCloud();
      ROS_WARN("Cleared PointCloud after rendering to openGL list. It will not be available for save/send.");
      return;
    }
  }
}


void GraphManager::sanityCheck(float thresh){ 
  thresh *=thresh; //squaredNorm
  QMutexLocker locker(&optimizer_mutex);
  EdgeSet::iterator edge_iter = optimizer_->edges().begin();
  for(int i =0;edge_iter != optimizer_->edges().end(); edge_iter++, i++) {
      g2o::EdgeSE3* myedge = dynamic_cast<g2o::EdgeSE3*>(*edge_iter);
      Eigen::Vector3d ev = myedge->measurement().translation();
      if(ev.squaredNorm() > thresh){
        optimizer_->removeEdge(myedge); 
      }
  }
}
void GraphManager::pruneEdgesWithErrorAbove(float thresh){
  QMutexLocker locker(&optimizer_mutex);
  EdgeSet::iterator edge_iter = optimizer_->edges().begin();
  for(int i =0;edge_iter != optimizer_->edges().end(); edge_iter++, i++) {
      g2o::EdgeSE3* myedge = dynamic_cast<g2o::EdgeSE3*>(*edge_iter);
      g2o::EdgeSE3::ErrorVector ev = myedge->error();
      if(ev.squaredNorm() > thresh){
        optimizer_->removeEdge(myedge); 
      }
  }
}
void GraphManager::printEdgeErrors(QString filename){
  QMutexLocker locker(&optimizer_mutex);
  std::fstream filestr;
  filestr.open (qPrintable(filename),  std::fstream::out );

  EdgeSet::iterator edge_iter = optimizer_->edges().begin();
  for(int i =0;edge_iter != optimizer_->edges().end(); edge_iter++, i++) {
      g2o::EdgeSE3* myedge = dynamic_cast<g2o::EdgeSE3*>(*edge_iter);
      g2o::EdgeSE3::ErrorVector ev = myedge->error();
      ROS_INFO_STREAM("Error Norm for edge " << i << ": " << ev.squaredNorm());
      filestr << "Error for edge " << i << ": " << ev.squaredNorm() << std::endl;
  }
  filestr.close();
}

void GraphManager::finishUp(){
  someone_is_waiting_for_me_ = true;
}
bool GraphManager::isBusy(){
  return (batch_processing_runs_ || process_node_runs_ );
}

double GraphManager::geodesicDiscount(g2o::HyperDijkstra& hypdij, const MatchingResult& mr){
    //Discount by geodesic distance to root node
    const g2o::HyperDijkstra::AdjacencyMap am = hypdij.adjacencyMap();
    g2o::VertexSE3* older_vertex = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(mr.edge.id1));
    double discount_factor = am.at(older_vertex).distance();
    discount_factor = discount_factor > 0.0? 1.0/discount_factor : 1.0;//avoid inf
    ROS_INFO("Discount weight for connection to Node %i = %f", mr.edge.id1, discount_factor);
    return discount_factor;
}

void GraphManager::toggleMapping(bool mappingOn){
  QMutexLocker locker(&optimizer_mutex);
  ROS_INFO_COND(mappingOn, "Switching mapping back on");
  ROS_INFO_COND(!mappingOn, "Switching mapping off: Localization continues");
  localization_only_ = !mappingOn;
  BOOST_FOREACH(VertexIDPair pair, optimizer_->vertices()) {
      g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3* >(pair.second);
      v->setFixed(localization_only_);
  }
}


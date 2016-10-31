#include <aslam_demo/aslam/aslam.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

namespace aslam {
AslamBase::AslamBase(ros::NodeHandle& n):
n_(n),
probability_map_(1,1,1,gtsam::Point2(0.0,0.0)),
occupancy_grid_(),
costmap2dros_("costmap",tf_listener_),
tf_listener_(ros::Duration(10000)),
local_planner_(std::make_shared<dwa_local_planner::DWAPlannerROS>()),
current_pose_(gtsam::Pose2(100.0,100.0,0.0)),
alpha_(1.0),
MotPrimFilename_("/home/sriramana/sbpl/matlab/mprim/pr2.mprim") {

  ROS_INFO_STREAM("A");
  local_planner_->initialize("local",&tf_listener_,&costmap2dros_);
  velocity_publisher_  = n_.advertise<geometry_msgs::Twist>("cmd_vel",1);
  vis_publisher = n_.advertise<visualization_msgs::MarkerArray>( "visualization_marker", 0 );
  ROS_INFO_STREAM("Costmap"<<(int)costmap2dros_.getCostmap()->getCharMap()[200]);

}

void AslamBase::updateFromOccMap(nav_msgs::OccupancyGrid& occupancy_grid,gtsam::Pose2 current_pose) {
  ROS_INFO_STREAM("Entered");
  occupancy_grid_  = occupancy_grid;
  ROS_INFO_STREAM("Occupancy grid updated");

  probability_map_ = mapping::ProbabilityMap(occupancy_grid);
  ROS_INFO_STREAM("Occupancy probabMap Updated");

  mainAslamAlgorithm();
}


void AslamBase::updateFromProbMap(mapping::ProbabilityMap& probability_map,gtsam::Pose2 current_pose) {
  ROS_INFO_STREAM("Entered OOO");

  probability_map_.reset(probability_map);
  ROS_INFO_STREAM("Occupancy grid updated");

  probability_map_.occupancyGrid(occupancy_grid_);
  ROS_INFO_STREAM("Occupancy probabMap Updated");
  gtsam::Point2 world_point(current_pose.x(),current_pose.y());
  gtsam::Point2 map_point = probability_map_.fromWorld(world_point);
  current_pose_ = gtsam::Pose2(map_point.x()*probability_map_.cellSize(),map_point.y()*probability_map_.cellSize(),current_pose.theta());

  mainAslamAlgorithm();
}


void AslamBase::MarkerConfig(visualization_msgs::Marker& marker,gtsam::Pose2& pose,int& id) {
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "aslam";
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = pose.x();
  marker.pose.position.y = pose.y();
  marker.pose.position.z = 0.0;
  tf::Quaternion q;
  q.setEuler(pose.theta(),0.0,0.0);
  tf::quaternionTFToMsg(q,marker.pose.orientation);

  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.scale.z = 0.01;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
}


void AslamBase::addToMarkerArray(visualization_msgs::MarkerArray& marker_array,gtsam::Pose2& pose2) {
  visualization_msgs::Marker marker;
  int id = marker_array.markers.size();
  MarkerConfig(marker,pose2,id);
  marker_array.markers.push_back(marker);
}


void AslamBase::driveRobot(rosTrajectory& trajectory) {
  //@todo: figure out where this comes from
//  setSPBLEnvfromOccupancyGrid(env_,occupancy_grid_,MotPrimFilename_);
//  planxythetalat(env_,start,goal,spbl_global_path_);
//  fromSPBLtoROSpath(spbl_global_path_,ros_global_path_);
  local_planner_->setPlan(trajectory);
  geometry_msgs::Twist cmd_vel;
  while(!local_planner_->isGoalReached()) {
    bool success = local_planner_->computeVelocityCommands(cmd_vel);
    velocity_publisher_.publish(cmd_vel);

  }
}

void AslamBase::mainAslamAlgorithm() {
  ROS_INFO_STREAM("main Aslam entered");
  vectorOfIndices frontier_indices,cluster_centers;
  getFrontierCells(occupancy_grid_,frontier_indices);
  ROS_INFO_STREAM("main Aslam entered");
  for(auto const iter:frontier_indices) {
    gtsam::Pose2 pose(iter.first*probability_map_.cellSize(),iter.second*probability_map_.cellSize(),0.0);
    addToMarkerArray(marker_array_,pose);
  }
  ROS_INFO_STREAM("Marker Array"<<marker_array_.markers.size());
  vis_publisher.publish(marker_array_);
  findFrontierClusters(frontier_indices,cluster_centers);
  ROS_INFO_STREAM("main Aslam entered"<<MotPrimFilename_);

  setSPBLEnvfromOccupancyGrid(env_,occupancy_grid_,MotPrimFilename_);
  spblTrajectoryList trajectory_list;
  for(auto const &iter: cluster_centers) {
    ROS_INFO_STREAM("Cluster Centers");

    spblTrajectory trajectory;
    gtsam::Pose2 goal(iter.first*probability_map_.cellSize(),iter.second*probability_map_.cellSize(),0.0); //@todo get good estimate of orientation
    ROS_INFO_STREAM("Coordinates"<<iter.first<<"\t"<<iter.second<<"\t"<<current_pose_.x()<<"\t"<<current_pose_.y());

    planxythetalat(env_,current_pose_,goal,trajectory);
    trajectory_list.push_back(trajectory);
  }
  ROS_INFO_STREAM("Traj Length"<<trajectory_list.size());
 /* spblTrajectory best_trajectory;
  selectTrajectory(probability_map_,trajectory_list,best_trajectory);
  rosTrajectory trajectory;
  fromSPBLtoROSpath(best_trajectory,trajectory);
  driveRobot(trajectory); */


}


void AslamBase::predictedMeasurement(mapping::ProbabilityMap& probability_map,spblTrajectory& trajectory,LaserScanList& predicted_scans) {
  std::vector<double> angles;//@todo figure the sensor information
  double laser_range = 3.5;
  double occupancy_probability = 0.8;
  for(auto const &pose: trajectory) {
    sensor_msgs::LaserScan laser_scan;
    gtsam::Point2 start_point = gtsam::Point2(pose.x,pose.y);
    double angle_offset = 0;//@todo change this to pose.theta
    for(auto const &angle: angles) {
      gtsam::Point2 end_point = probability_map.findEndPoints(start_point,laser_range,angle + angle_offset);
      std::vector<mapping::ProbabilityMap::LineCell> line_cell = probability_map.line(start_point,end_point);
      bool is_special = false;
      double expected_range = 0.0;
      //@todo Make it better. Now its shitty
      for(auto const &ele: line_cell) {
        double value = probability_map.at(ele.row,ele.col);
        if(!is_special && value > occupancy_probability) {
          gtsam::Point2 mid_point((ele.start.x() + ele.end.x())/2,(ele.start.y() + ele.end.y())/2);
          expected_range = start_point.dist(mid_point);
        }
      }
      laser_scan.ranges.push_back(expected_range);
    }
    predicted_scans.push_back(laser_scan);
  }
}

void AslamBase::getProbabilityMaps(const mapping::ProbabilityMap& probability_map,spblTrajectory& trajectory,LaserScanList& measurements,ProbabilityMaps& probability_maps) {
  mapping::ProbabilityMap current_map(probability_map);
  mapping::sensor_models::LaserScanModel laser_scan_model;
  gtsam::Pose3 base_T_laser; //@todo get this from somewhere
  if (trajectory.size() != measurements.size()) ROS_ERROR("Size not Equal");
  for(size_t i = 0;i < trajectory.size();i++) {
    auto pose = trajectory[i];
    auto measurement  = measurements[i];
    gtsam::Pose2 world_T_base(pose.x,pose.y,pose.theta);
    laser_scan_model.updateMap(current_map,measurement,world_T_base,base_T_laser);
    probability_maps.push_back(current_map);
  }
}

double AslamBase::shannonEntropy(const mapping::ProbabilityMap& probability_map,spblTrajectory& trajectory,LaserScanList& predicted_scans) {
  double entropy = 0.0;
  //@todo Use differential approach later. This is too fucking naive
  for(size_t row = 0;row < probability_map.rows();row++)
    for(size_t col = 0;col < probability_map.cols(); col++) {
      double probability = probability_map.at(row,col);
      entropy += probability*log(probability) + (1 - probability)*log(1 - probability);
    }
  return(-entropy);
}


double AslamBase::renyiEntopy(mapping::ProbabilityMap& probability_map,spblTrajectory& trajectory,LaserScanList& predicted_scans) {
  double entropy;
  double alpha = 1.0;
  for(size_t row = 0;row < probability_map.rows();row++)
     for(size_t col = 0;col < probability_map.cols(); col++) {
       double probability = probability_map.at(row,col);
       entropy += pow(probability,alpha);
     }
  return(log(entropy)/(1 - alpha));

}

void AslamBase::selectTrajectory(mapping::ProbabilityMap& probability_map, spblTrajectoryList &trajectory_list,spblTrajectory best_trajectory) {
  double best_score = -100.0;
  size_t max_index = 0.0;
  for (size_t index = 0;index < trajectory_list.size(); index++) {
    double score = utilityOfTrajectory(probability_map,trajectory_list[index]);
    if(best_score < score) {
      best_score = score;
      max_index = index;
    }
  }
  best_trajectory = trajectory_list[max_index];
}


double AslamBase::utilityOfTrajectory(mapping::ProbabilityMap& probability_map, spblTrajectory &trajectory) {
  LaserScanList predicted_scans;
  predictedMeasurement(probability_map,trajectory,predicted_scans);
  ProbabilityMaps probability_maps;
  getProbabilityMaps(probability_map,trajectory,predicted_scans,probability_maps);
  double utility = 0.0;
  for(auto const &prob_map: probability_maps) {
    utility += shannonEntropy(prob_map,trajectory,predicted_scans);
  }
  return(utility);
}

/*

void AslamBase::setCostMapfromOccGrid(nav_msgs::OccupancyGrid& occupancy_grid,costmap_2d::Costmap2DROS& costmap2dros){
  size_t height = occupancy_grid.info.height, width =  occupancy_grid.info.width;
  double resolution = occupancy_grid.info.resolution;
  double origin_x = occupancy_grid.info.origin.position.x, origin_y = occupancy_grid.info.origin.position.y;
  unsigned char data = occupancy_grid.data;
  costmap_2d::Costmap2D costmap2d(height,width,resolution,origin_x,origin_y);
  costmap2dros.getCostmap()->Costmap2D = costmap2d;
}
*/


void AslamBase::createFootprint(std::vector<sbpl_2Dpt_t>& perimeter) {
    sbpl_2Dpt_t pt_m;
    double halfwidth = 0.01;
    double halflength = 0.01;
    pt_m.x = -halflength;
    pt_m.y = -halfwidth;
    perimeter.push_back(pt_m);
    pt_m.x = halflength;
    pt_m.y = -halfwidth;
    perimeter.push_back(pt_m);
    pt_m.x = halflength;
    pt_m.y = halfwidth;
    perimeter.push_back(pt_m);
    pt_m.x = -halflength;
    pt_m.y = halfwidth;
    perimeter.push_back(pt_m);
}


void AslamBase::setSPBLEnvfromOccupancyGrid(EnvironmentNAVXYTHETALAT& env, nav_msgs::OccupancyGrid& occupancy_grid, char* MotPrimFilename) {
  int width = occupancy_grid.info.width;
  int height = occupancy_grid.info.height;
  unsigned char mapdata[height*width];
  for(size_t i = 0;i < height*width;i++) {
    int data = occupancy_grid.data[i];
   // size_t row_occ = i/occupancy_grid.info.width,col_occ = i%occupancy_grid.info.width;
    if(data == -1 ) data = 127;
    mapdata[i] = (uchar)(data);
  }
  double startx = 0.0,starty = 0.0,starttheta = 0.0,goalx = 0.0,goaly = 0.0,goaltheta = 0.0;
  double goaltol_x = 0.001,goaltol_y = 0.001,goaltol_theta = 0.001;
  std::vector< sbpl_2Dpt_t > perimeterptsV;
  createFootprint(perimeterptsV);
  double cellsize_m = occupancy_grid.info.resolution;
  double nominalvel_mpersecs = 0.1;
  double timetoturn45degsinplace_secs = 0.1;
  unsigned char obsthresh = 100;
  FILE* fMotPrim = fopen(MotPrimFilename, "r");
  if(fMotPrim == NULL) ROS_INFO("This is it");
  env.InitializeEnv(width,height,mapdata,startx,starty,starttheta,goalx,goaly,goaltheta,goaltol_x,goaltol_y,goaltol_theta,perimeterptsV,cellsize_m,nominalvel_mpersecs,timetoturn45degsinplace_secs,obsthresh,MotPrimFilename);
}

void initializePlanner(std::shared_ptr<SBPLPlanner>& planner,
                       EnvironmentNAVXYTHETALAT& env,
                       int start_id, int goal_id,
                       double initialEpsilon,
                       bool bsearchuntilfirstsolution) {
    // work this out later, what is bforwardsearch?
    bool bsearch = false;
    planner = std::make_shared<ARAPlanner>(&env, bsearch);

    // set planner properties
    if (planner->set_start(start_id) == 0) {
        printf("ERROR: failed to set start state\n");
        throw new SBPL_Exception();
    }
    if (planner->set_goal(goal_id) == 0) {
        printf("ERROR: failed to set goal state\n");
        throw new SBPL_Exception();
    }
    planner->set_initialsolution_eps(initialEpsilon);
    planner->set_search_mode(bsearchuntilfirstsolution);
}

int runPlanner(std::shared_ptr<SBPLPlanner>&  planner, int allocated_time_secs,
               std::vector<int>& solution_stateIDs) {
    int bRet = planner->replan(allocated_time_secs, &solution_stateIDs);

    if (bRet)
        printf("Solution is found\n");
    else
        printf("Solution does not exist\n");
    return bRet;
}

void writeSolution(EnvironmentNAVXYTHETALAT& env, std::vector<int> solution_stateIDs,
    const char* filename) {

  std::string discrete_filename(std::string(filename) + std::string(".discrete"));
  FILE* fSol_discrete = fopen(discrete_filename.c_str(), "w");
  FILE* fSol = fopen(filename, "w");
  if (fSol == NULL) {
      printf("ERROR: could not open solution file\n");
      throw SBPL_Exception();
  }

  // write the discrete solution to file
  for (size_t i = 0; i < solution_stateIDs.size(); i++) {
      int x, y, theta;
      env.GetCoordFromState(solution_stateIDs[i], x, y, theta);
      double cont_x, cont_y, cont_theta;
      cont_x = DISCXY2CONT(x, 0.1);
      cont_y = DISCXY2CONT(y, 0.1);
      cont_theta = DiscTheta2Cont(theta, 16);
      fprintf(fSol_discrete, "%d %d %d\n", x, y, theta);
  }
  fclose(fSol_discrete);

  // write the continuous solution to file
  std::vector<sbpl_xy_theta_pt_t> xythetaPath;
  env.ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &xythetaPath);
  for (unsigned int i = 0; i < xythetaPath.size(); i++) {
      fprintf(fSol, "%.3f %.3f %.3f\n", xythetaPath.at(i).x,
                                        xythetaPath.at(i).y,
                                        xythetaPath.at(i).theta);
  }
  fclose(fSol);

}

void getSBPLpathfromID(EnvironmentNAVXYTHETALAT& env, const std::vector<int>& solution_stateIDs, std::vector<sbpl_xy_theta_pt_t> &xythetaPath) {
  for (size_t i = 0; i < solution_stateIDs.size(); i++) {
        int x, y, theta;
        env.GetCoordFromState(solution_stateIDs[i], x, y, theta);
        double cont_x, cont_y, cont_theta;
        cont_x = DISCXY2CONT(x, 0.1);
        cont_y = DISCXY2CONT(y, 0.1);
        cont_theta = DiscTheta2Cont(theta, 16);
        xythetaPath.push_back(sbpl_xy_theta_pt_t(cont_x,cont_y,cont_theta));
   }
}


void AslamBase::planxythetalat(EnvironmentNAVXYTHETALAT& env,gtsam::Pose2& start,gtsam::Pose2& goal,std::vector<sbpl_xy_theta_pt_t> &xythetaPath) {
  int start_id = env.SetStart(start.x(), start.y(), start.theta());
  int goal_id = env.SetGoal(goal.x(), goal.y(), goal.theta());

  double initialEpsilon = 3.0;
  bool bsearchuntilfirstsolution = false;
  initializePlanner(planner_,env,start_id,goal_id,initialEpsilon,bsearchuntilfirstsolution);

  std::vector<int> solution_stateIDs;
  double allocated_time_secs = 10.0;
  runPlanner(planner_,allocated_time_secs,solution_stateIDs);

  //env.ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &xythetaPath);
  getSBPLpathfromID(env,solution_stateIDs,xythetaPath);

}


void AslamBase::fromSPBLtoROSpath(std::vector<sbpl_xy_theta_pt_t> &xythetaPath, std::vector< geometry_msgs::PoseStamped > &plan) {
  for(auto const &iter: xythetaPath) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose.position.x = iter.x;
    pose_stamped.pose.position.y = iter.y;
    pose_stamped.pose.position.z = 0.0;

    tf::Quaternion q;
    q.setEuler(iter.theta,0.0,0.0);
    tf::quaternionTFToMsg(q,pose_stamped.pose.orientation);

    plan.push_back(pose_stamped);
  }
}



void AslamBase::getFrontierCells(nav_msgs::OccupancyGrid& occupancy_grid,std::vector<std::pair<int,int> >& frontier_indices) {
  ROS_INFO_STREAM("Frontier Cells Entered");

  size_t height = occupancy_grid.info.height;
  size_t width = occupancy_grid.info.width;
  size_t row = 0, col = 0;
  auto data = occupancy_grid.data;
  std::vector<std::pair<int,int> > indices_offset;

  indices_offset.push_back(std::make_pair(-1,-1));
  indices_offset.push_back(std::make_pair(-1,0));
  indices_offset.push_back(std::make_pair(-1,1));
  indices_offset.push_back(std::make_pair(0,-1));
  indices_offset.push_back(std::make_pair(0,1));
  indices_offset.push_back(std::make_pair(1,-1));
  indices_offset.push_back(std::make_pair(1,0));
  indices_offset.push_back(std::make_pair(1,1));

  for (size_t i = 0;i < height*width;i++) {
    int current = data[i];
    if(!(current < 127)) continue;
    row = i/width;
    col = i%width;
    bool occupied_present = false,unknown_present = false;
    for(auto const iter: indices_offset) {
      size_t new_row = (row + iter.first);
      size_t new_col = (col + iter.second);
      if (new_row < 0 || new_row >= height || new_col < 0 || new_col >= width) continue;

      int value = data[height*new_row + new_col];
      if (value > 127) occupied_present = true;
      if (value == 127) unknown_present = true;

    }
    if(!occupied_present && unknown_present) frontier_indices.push_back(std::make_pair(row,col));
  }
}

void AslamBase::findFrontierClusters(vectorOfIndices& frontier_indices,vectorOfIndices& cluster_centers) {
  ROS_INFO_STREAM("Frontier Clusters Entered");
  int numPoints = frontier_indices.size();
  cv::Mat points(numPoints,2,CV_32FC1),labels,centers;
  int cluster_count = 2;
  int index = 0;
  for(auto const iter: frontier_indices) {
    points.at<float>(index,0) = (float)(iter.first);
    points.at<float>(index,1) = (float)(iter.second);
    index++;
  }


  //ROS_INFO_STREAM("Point"<<points.size());
  double temp = cv::kmeans(points, cluster_count, labels,cv::TermCriteria(cv::TermCriteria::EPS +cv::TermCriteria::COUNT, 1000,0.01),3, cv::KMEANS_PP_CENTERS , centers);
  //ROS_INFO_STREAM("Temp"<<temp);
  //ROS_INFO_STREAM("Labels"<<labels.size()<<"\tCenters: "<<centers.size());
  for(int row = 0;row < centers.rows;row++) {
    //  ROS_INFO_STREAM("Centers:"<<centers.at<float>(row,0)<<"\t"<<centers.at<float>(row,1));
      cluster_centers.push_back(std::make_pair(centers.at<float>(row,0),centers.at<float>(row,1)));
  }
}

AslamBase::~AslamBase() {

}
};

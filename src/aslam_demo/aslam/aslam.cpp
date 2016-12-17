#include <aslam_demo/aslam/aslam.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>


namespace aslam {
AslamBase::AslamBase(ros::NodeHandle& n,std::string base_name,std::string laser_link,ros::Time& time):
n_(n),
probability_map_(1,1,1,gtsam::Point2(0.0,0.0)),
occupancy_grid_(),
costmap2dros_("costmap",tf_listener_),
tf_listener_(ros::Duration(1000000)),

//local_planner_(std::make_shared<dwa_local_planner::DWAPlannerROS>()),
current_pose_(gtsam::Pose2(100.0,100.0,0.0)),
alpha_(1.0),
MotPrimFilename_("/home/sriramana/sbpl/matlab/mprim/pr2.mprim"),
base_name_(base_name),
laser_link_(laser_link),
time_ptr_(&time) {

  ROS_INFO_STREAM("AslamBase Object"<<base_name_<<"\t"<<laser_link_);

 // local_planner_->initialize("local",&tf_listener_,&costmap2dros_);
  local_planner_.initialize("local",&tf_listener_,&costmap2dros_);
  velocity_publisher_  = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
  vis_publisher = n_.advertise<visualization_msgs::MarkerArray>( "visualization_marker", 0 );
  plan_publisher = n_.advertise<nav_msgs::Path>( "global_path", 0 );
  map_publisher = n_.advertise<nav_msgs::OccupancyGrid>( "dmap", 0 );

  tf::StampedTransform ts;
  tf::Transform tr;
  try {
      tf_listener_.lookupTransform(base_name_,laser_link_,
                                   ros::Time(0), ts);
      tr = ts;
      fromTftoGtsamPose(base_T_laser_,tr);
  }
  catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
   }
  costmap2dros_.start();
  initialized_ = true;
}


void AslamBase::fromTftoGtsamPose(gtsam::Pose3 &pose3, const tf::Transform &transform) {
  tf::Vector3 translation = transform.getOrigin();
  tf::Matrix3x3 rotation = transform.getBasis();
  gtsam::Point3 trans(translation.getX(),translation.getY(),translation.getZ());
  gtsam::Rot3 rot(rotation[0][0],rotation[0][1],rotation[0][2],rotation[1][0],rotation[1][1],rotation[1][2],rotation[2][0],rotation[2][1],rotation[2][2]);
  gtsam::Pose3 new_pose(rot,trans);
  pose3 = new_pose;

}


void fromGtsamPose2toTf(const gtsam::Pose2 &pose2, tf::Transform &transform) {

  tf::Vector3 translation(pose2.x(),pose2.y(),0);
  tf::Matrix3x3 rotation;
  rotation.setRPY(0.0,0.0,pose2.theta());

  transform.setOrigin(translation);
  transform.setBasis(rotation);

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
  //costmap2dros_.resetLayers();
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
  marker.header.frame_id = "map";
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

  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.01;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
}

void resetMarkerArray(visualization_msgs::MarkerArray& marker_array) {
  for(auto &iter: marker_array.markers) {
    iter.header.stamp = ros::Time::now();
    iter.header.frame_id = "map";
  }
}


void AslamBase::addToMarkerArray(visualization_msgs::MarkerArray& marker_array,gtsam::Pose2& pose2) {
  visualization_msgs::Marker marker;
  int id = marker_array.markers.size();
  MarkerConfig(marker,pose2,id);
  marker_array.markers.push_back(marker);
}



void AslamBase::driveRobot(rosTrajectory& trajectory) {
  //@todo: figure out where this comes from

  while(!local_planner_.isInitialized())  {
    local_planner_.initialize("local",&tf_listener_,&costmap2dros_);
  }
  nav_msgs::Path path;
  path.header.frame_id = base_name_;
  path.header.stamp = ros::Time::now();
  for(auto const iter: trajectory) {
    path.poses.push_back(iter);
  }
  plan_publisher.publish(path);
  bool plan_set = local_planner_.setPlan(trajectory);
  int stuck_count = 0;
  double thresh = 0.1;
  gtsam::Point2 point1(0.0,0.0),point2(0.0,0.0);
  while(!local_planner_.isGoalReached() && plan_set) {
    geometry_msgs::Twist cmd_vel;
    bool plan_set = local_planner_.setPlan(trajectory);


    auto goall = *trajectory.rbegin();
    try {
             tf_listener_.waitForTransform(base_name_,"map",
                                          ros::Time::now(),ros::Duration(100));
    }
         catch (tf::TransformException &ex) {
             ROS_ERROR("%s",ex.what());
             ros::Duration(1.0).sleep();
    }
    tf::Stamped<tf::Pose> pose;
    costmap2dros_.getRobotPose(pose);
    tf::StampedTransform ts;
    ros::Time curr_time = ros::Time::now();
    point1 = gtsam::Point2(pose.getOrigin().x(),pose.getOrigin().y());
    if(point2.dist(point1) < thresh) stuck_count++;

    ROS_INFO_STREAM("Pose"<< pose.getOrigin().x() <<"\t"<< pose.getOrigin().y()<<"\t"<<pose.getRotation()<<"\t"<<goall.pose.position.x<<"\t"<<goall.pose.position.y<<"\t"<<goall.pose.orientation);
    bool success = local_planner_.computeVelocityCommands(cmd_vel);
    ROS_INFO_STREAM("Pose"<< local_planner_.isGoalReached());

  //  bool success = local_planner_.dwaComputeVelocityCommands(pose,cmd_vel);
  //  while(--buffer) {
      if (success) velocity_publisher_.publish(cmd_vel);
      point2 = point1;
      if(stuck_count > 10) {
        int buffer = 10000;
        while(--buffer) {
          float rndm = rand()/(float)RAND_MAX;
          cmd_vel.linear.x = -rndm;
          cmd_vel.linear.y = -rndm;
          cmd_vel.angular.z = 0.0;

          velocity_publisher_.publish(cmd_vel);
          stuck_count = 0;
        }
      }
  //  }
     // ros::Duration(10.0).sleep();
    while(((curr_time - *time_ptr_) > ros::Duration(0.1)));
    resetMarkerArray(marker_array_);
    vis_publisher.publish(marker_array_);

  }
}

void AslamBase::fromGtsamPose2toTfPose(gtsam::Pose2& pose2,tf::Pose& tf_pose) {
  tf_pose.setOrigin(tf::Vector3(pose2.x(),pose2.y(),0.0));
  tf::Quaternion q;
  q.setEuler(pose2.theta(),0.0,0.0);
  tf_pose.setRotation(q);
}

void AslamBase::fromGtsamPose2toROS(gtsam::Pose2& pose2,geometry_msgs::Pose& pose) {
  pose.position.x = pose2.x();
  pose.position.y = pose2.y();
  pose.position.z = 0.0;
  tf::Quaternion q;
  q.setEuler(pose2.theta(),0.0,0.0);
  tf::quaternionTFToMsg(q,pose.orientation);

}

void AslamBase::mainAslamAlgorithm() {
  ROS_INFO_STREAM("main Aslam entered");
  vectorOfIndices frontier_indices,cluster_centers;
  getFrontierCells(occupancy_grid_,frontier_indices);
  ROS_INFO_STREAM("main Aslam entered");
  for(auto const iter:frontier_indices) {
    gtsam::Point2 world_point = probability_map_.toWorld(gtsam::Point2(iter.first,iter.second));
    gtsam::Pose2 pose(world_point.x(),world_point.y(),0.0);
    addToMarkerArray(marker_array_,pose);
  }
  ROS_INFO_STREAM("Marker Array"<<marker_array_.markers.size());
 //c vis_publisher.publish(marker_array_);
  //while(1)  vis_publisher.publish(marker_array_);
  findFrontierClusters(frontier_indices,cluster_centers);
  ROS_INFO_STREAM("main Aslam entered"<<MotPrimFilename_);
  EnvironmentNAVXYTHETALAT env;
  planner_init_ = false;
  setSPBLEnvfromOccupancyGrid(env,occupancy_grid_,MotPrimFilename_);
  spblTrajectoryList trajectory_list;
  for(auto const &iter: cluster_centers) {
    ROS_INFO_STREAM("Cluster Centers");
    spblTrajectory trajectory;
    gtsam::Point2 world_point = probability_map_.toWorld(gtsam::Point2(iter.first,iter.second));
    gtsam::Pose2 pose(world_point.x(),world_point.y(),0.0);
    gtsam::Pose2 goal(world_point.x() - probability_map_.origin().x(),world_point.y() - probability_map_.origin().y(),0.0); //@todo get good estimate of orientation
    addToMarkerArray(marker_array_,pose);
  //  gtsam::Pose2 goal(16,15.0,0); //@todo Inflate obstacles and remove invalid configuration
    std::vector<int> solution_stateIDs;
    planxythetalat(env,current_pose_,goal,trajectory,solution_stateIDs,planner_init_);
    plotsbplTrajectory(trajectory);
    trajectory_list.push_back(trajectory);
  }
  vis_publisher.publish(marker_array_);

  ROS_INFO_STREAM("Traj Length"<<trajectory_list.size());
  spblTrajectory best_trajectory;
  selectTrajectory(probability_map_,trajectory_list,best_trajectory);
  rosTrajectory ros_trajectory;
  fromSPBLtoROSpath(best_trajectory,ros_trajectory);
  ROS_INFO_STREAM("Traj len"<<ros_trajectory.size()<<"\t"<<best_trajectory.size());
 // while(1);
  driveRobot(ros_trajectory);
  marker_array_.markers.clear();


}


void AslamBase::plotsbplTrajectory(spblTrajectory& trajectory) {
  ROS_INFO_STREAM("Trajectory"<<trajectory.size());
  for(auto const pose: trajectory) {

    gtsam::Pose2 new_pose = probability_map_.fromSBPL(pose);
    addToMarkerArray(marker_array_,new_pose);
  }
  vis_publisher.publish(marker_array_);
  ROS_INFO_STREAM(trajectory.size());
}


void AslamBase::predictedMeasurement(mapping::ProbabilityMap& probability_map,spblTrajectory& trajectory,LaserScanList& predicted_scans) {
  std::vector<double> angles;//@todo figure the sensor information
  double angle_min = -0.521567881107,angle_max = 0.524276316166,angle_increment = 0.00163668883033;
  double angle = angle_min;
  while(angle <= angle_max) {
    angles.push_back(angle);
    angle += angle_increment;
  }
  double laser_range = 3.5;
  double occupancy_probability = 0.8;
  for(auto const &pose: trajectory) {
    sensor_msgs::LaserScan laser_scan;
    laser_scan.angle_min = angle_min;
    laser_scan.angle_max = angle_max;
    laser_scan.angle_increment = angle_increment;
    laser_scan.header.frame_id = laser_link_;
    laser_scan.range_max = 3.5;

    gtsam::Point2 sbpl_point = gtsam::Point2(pose.x,pose.y);
    gtsam::Point2 start_point = probability_map.fromSBPL(sbpl_point);
    double angle_offset = pose.theta;//@todo change this to pose.theta
    for(auto const &angle: angles) {
      gtsam::Point2 end_point = probability_map.findEndPoints(start_point,laser_range,angle + angle_offset);
      std::vector<mapping::ProbabilityMap::LineCell> line_cell = probability_map.line(start_point,end_point);

      bool is_special = false;
      double expected_range = laser_range;
      //@todo Make it better. Now its shitty
      for(auto const &ele: line_cell) {
        double value = probability_map.at(ele.row,ele.col);
        if(value > occupancy_probability) {
          gtsam::Point2 mid_point((ele.start.x() + ele.end.x())/2,(ele.start.y() + ele.end.y())/2);
          expected_range = start_point.dist(mid_point);
        }
    //   ROS_INFO_STREAM("ER\t" << expected_range);
      }
      if (expected_range == 0.0) expected_range += 0.000001;
      laser_scan.ranges.push_back(expected_range);
    }
    predicted_scans.push_back(laser_scan);

  }
}

void AslamBase::getProbabilityMaps(const mapping::ProbabilityMap& probability_map,spblTrajectory& trajectory,LaserScanList& measurements,ProbabilityMaps& probability_maps) {
  mapping::ProbabilityMap current_map(probability_map);
  current_map.calcShannonEntropy();
  mapping::sensor_models::LaserScanModel laser_scan_model(0.05,true);
  //gtsam::Pose3 base_T_laser = gtsam::Pose3::identity(); //@todo get this from somewhere
  if (trajectory.size() != measurements.size()) ROS_ERROR("Size not Equal");
  for(size_t i = 0;i < trajectory.size();i++) {
    gtsam::Pose2 sbpl_pose = gtsam::Pose2(trajectory[i].x,trajectory[i].y,trajectory[i].theta);
    gtsam::Pose2 world_T_base = probability_map.fromSBPL(sbpl_pose);
    auto measurement  = measurements[i];
   // gtsam::Pose2 world_T_base(pose.x,pose.y,pose.theta);
  //   ROS_INFO_STREAM("Measurement"<<measurement);
    laser_scan_model.updateMap(current_map,measurement,world_T_base,base_T_laser_);
    probability_maps.push_back(current_map);
    nav_msgs::OccupancyGrid curr_map,curr_map_pub;
    current_map.occupancyGrid(curr_map);
    current_map.getPublishableMap(curr_map,curr_map_pub);
    map_publisher.publish(curr_map_pub);
    ROS_INFO_STREAM("CR"<<current_map.getShannonEntropy());
  }
}

double AslamBase::shannonEntropy(const mapping::ProbabilityMap& probability_map,spblTrajectory& trajectory,LaserScanList& predicted_scans) {
 /* double entropy = 0.0;
  //@todo Use differential approach later. This is too fucking naive
  for(size_t row = 0;row < probability_map.rows();row++)
    for(size_t col = 0;col < probability_map.cols(); col++) {
      double probability = probability_map.at(row,col);
      entropy += probability*log(probability) + (1 - probability)*log(1 - probability);
    }
  return(-entropy);*/
  return probability_map.getShannonEntropy();
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

void AslamBase::selectTrajectory(mapping::ProbabilityMap& probability_map, spblTrajectoryList &trajectory_list,spblTrajectory& best_trajectory) {
  double best_score = -100.0;
  size_t max_index = 0;
  for (size_t index = 0;index < trajectory_list.size(); index++) {
    double score = utilityOfTrajectory(probability_map,trajectory_list[index]);
    ROS_INFO_STREAM("Score\t"<<score);
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
  ROS_INFO_STREAM("Predicted Scans\t"<<predicted_scans.size());

  ProbabilityMaps probability_maps;
  getProbabilityMaps(probability_map,trajectory,predicted_scans,probability_maps);
  ROS_INFO_STREAM("Probability map\t"<<probability_maps.size());

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
    double halfwidth = 0.1;
    double halflength = 0.1;
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
    if(data < 200) data = 0;
    mapdata[i] = data;
  }
  double startx = 0.0,starty = 0.0,starttheta = 0.0,goalx = 0.0,goaly = 0.0,goaltheta = 0.0;
  double goaltol_x = 0.001,goaltol_y = 0.001,goaltol_theta = 0.001;
  std::vector< sbpl_2Dpt_t > perimeterptsV;
  createFootprint(perimeterptsV);
  double cellsize_m = occupancy_grid.info.resolution;
  double nominalvel_mpersecs = 0.1;
  double timetoturn45degsinplace_secs = 0.1;
  unsigned char obsthresh = 200;
  FILE* fMotPrim = fopen(MotPrimFilename, "r");
  if(fMotPrim == NULL) ROS_INFO("This is it");
  env.InitializeEnv(width,height,mapdata,startx,starty,starttheta,goalx,goaly,goaltheta,goaltol_x,goaltol_y,goaltol_theta,perimeterptsV,cellsize_m,nominalvel_mpersecs,timetoturn45degsinplace_secs,obsthresh,MotPrimFilename);
}

void AslamBase::initPlanner(std::shared_ptr<SBPLPlanner>& planner,EnvironmentNAVXYTHETALAT& env) {
  bool bsearch = false;
  planner = std::make_shared<ARAPlanner>(&env, bsearch);
  planner_init_ = true;
}

void initializePlanner(std::shared_ptr<SBPLPlanner>& planner,
                       EnvironmentNAVXYTHETALAT& env,
                       int start_id, int goal_id,
                       double initialEpsilon,
                       bool bsearchuntilfirstsolution) {
    ROS_INFO_STREAM("Initialize Planner");
    // work this out later, what is bforwardsearch?
   /* bool bsearch = false;
    planner = std::make_shared<ARAPlanner>(&env, bsearch);
    ROS_INFO_STREAM("Id"<<(int)start_id<<"\t"<<(int)goal_id);
    env.PrintState(start_id, true, stdout);*/

    // set planner properties
    if (planner->set_start(start_id) == 0) {
        ROS_ERROR("ERROR: failed to set start state\n");
        throw new SBPL_Exception();
    }
    if (planner->set_goal(goal_id) == 0) {
        ROS_ERROR("ERROR: failed to set goal state\n");
        throw new SBPL_Exception();
    }
    planner->set_initialsolution_eps(initialEpsilon);
    planner->set_search_mode(bsearchuntilfirstsolution);
}

int runPlanner(std::shared_ptr<SBPLPlanner>&  planner, int allocated_time_secs,
               std::vector<int>& solution_stateIDs) {
    ROS_INFO_STREAM("Run Planner");

    int bRet = planner->replan(allocated_time_secs, &solution_stateIDs);

    if (bRet)
        printf("Solution is found\n");
    else
        printf("Solution does not exist\n");
    return bRet;
}


void getSBPLpathfromID(EnvironmentNAVXYTHETALAT& env,mapping::ProbabilityMap& probability_map, const std::vector<int>& solution_stateIDs, std::vector<sbpl_xy_theta_pt_t> &xythetaPath) {
  for (size_t i = 0; i < solution_stateIDs.size(); i++) {
        int x, y, theta;
        env.GetCoordFromState(solution_stateIDs[i], x, y, theta);
        double cont_x, cont_y, cont_theta;
        cont_x = DISCXY2CONT(x,probability_map.cellSize());
        cont_y = DISCXY2CONT(y, probability_map.cellSize());
        cont_theta = DiscTheta2Cont(theta, 16);
        xythetaPath.push_back(sbpl_xy_theta_pt_t(cont_x,cont_y,cont_theta));
   }
}


void AslamBase::planxythetalat(EnvironmentNAVXYTHETALAT& env,gtsam::Pose2& start,gtsam::Pose2& goal,std::vector<sbpl_xy_theta_pt_t> &xythetaPath,  std::vector<int>& solution_stateIDs,bool initialized) {
  int start_id = env.SetStart(start.x(), start.y(), start.theta());
  int goal_id = env.SetGoal(goal.x(), goal.y(), goal.theta());

  double initialEpsilon = 3.0;
  bool bsearchuntilfirstsolution = false;
  if(!planner_init_) initPlanner(planner_,env);

  initializePlanner(planner_,env,start_id,goal_id,initialEpsilon,bsearchuntilfirstsolution);

  double allocated_time_secs = 10.0;
  runPlanner(planner_,allocated_time_secs,solution_stateIDs);

  //env.ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &xythetaPath);
  getSBPLpathfromID(env,probability_map_,solution_stateIDs,xythetaPath);
  planner_->force_planning_from_scratch_and_free_memory();


}


void AslamBase::fromSPBLtoROSpath(std::vector<sbpl_xy_theta_pt_t> &xythetaPath, std::vector< geometry_msgs::PoseStamped > &plan) {
  double offset = 0.0;
  for(auto const &iter: xythetaPath) {
    gtsam::Pose2 pose = probability_map_.fromSBPL(iter);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now() + ros::Duration(offset);
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose.position.x = pose.x();
    pose_stamped.pose.position.y = pose.y();
    pose_stamped.pose.position.z = 0.0;

    tf::Quaternion q;
    q.setEuler(pose.theta(),0.0,0.0);
    tf::quaternionTFToMsg(q,pose_stamped.pose.orientation);
    ROS_INFO_STREAM("Pse Stamped"<<pose_stamped);
    plan.push_back(pose_stamped);
    offset += 1.01;
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

      int value = data[width*new_row + new_col];
      if (value > 127) occupied_present = true;
      if (value == 127) unknown_present = true;

    }
    if(!occupied_present && unknown_present) frontier_indices.push_back(std::make_pair(col,row));
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

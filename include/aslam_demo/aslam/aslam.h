
#define ROS

#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>


#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <aslam_demo/mapping/map_processing.h>
#include <aslam_demo/mapping/probability_map.h>
#include <aslam_demo/factors/key_generator.h>
#include <aslam_demo/factors/laser_scan_factor.h>

#include <aslam_demo/mapping/csm_processing.h>
#include <aslam_demo/mapping/optimization_processing.h>
#include <aslam_demo/mapping/laserscan_processing.h>
#include <aslam_demo/mapping/odometry_processing.h>
#include <aslam_demo/mapping/sensor_models.h>





#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>

#include <gtsam_ros/gtsam_ros.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>

#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetModelStateRequest.h>
#include <gazebo_msgs/GetModelStateResponse.h>


#include <costmap_2d/costmap_2d_ros.h>
#include <dwa_local_planner/dwa_planner_ros.h>
#include <laser_geometry/laser_geometry.h>

#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

#include <kdtree++/kdtree.hpp>
#include <sbpl/headers.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace aslam {

class AslamBase {
  typedef std::vector<sbpl_xy_theta_pt_t> spblTrajectory;
  typedef std::vector<spblTrajectory> spblTrajectoryList;
  typedef std::vector< geometry_msgs::PoseStamped > rosTrajectory;
  typedef std::vector<rosTrajectory> rosTrajectoryList;
  typedef std::vector<std::pair<int,int> > vectorOfIndices;
  typedef std::vector<sensor_msgs::LaserScan> LaserScanList;
  typedef std::vector<mapping::ProbabilityMap> ProbabilityMaps;

public:

  AslamBase(ros::NodeHandle& n,std::string base_name);
  ros::NodeHandle n_;
  tf::TransformListener tf_listener_;

  costmap_2d::Costmap2DROS costmap2dros_;

  std::string base_name_;
  nav_msgs::OccupancyGrid occupancy_grid_;
  mapping::ProbabilityMap probability_map_;
  EnvironmentNAVXYTHETALAT env_;
  std::shared_ptr<SBPLPlanner> planner_;
  dwa_local_planner::DWAPlannerROS local_planner_;
  //std::shared_ptr<dwa_local_planner::DWAPlannerROS> local_planner_;
  spblTrajectory spbl_global_path_;
  rosTrajectory ros_global_path_;
  char* MotPrimFilename_;
  gtsam::Pose2 current_pose_;
  bool planner_init_ = false;


  gtsam::Pose3 base_T_laser_;
  ros::Publisher velocity_publisher_;
  ros::Publisher vis_publisher;
  visualization_msgs::MarkerArray marker_array_;
  //All interface stuff
  void updateFromProbMap(mapping::ProbabilityMap& probability_map,gtsam::Pose2 current_pose);
  void updateFromOccMap(nav_msgs::OccupancyGrid& occupancy_grid,gtsam::Pose2 current_pose);
  void fromTftoGtsamPose(gtsam::Pose3 &pose3, const tf::Transform &transform);
  void fromGtsamPose2toROS(gtsam::Pose2& pose2,geometry_msgs::Pose& pose);



  //spbl stuff
  void createFootprint(std::vector<sbpl_2Dpt_t>& perimeter);
  void setSPBLEnvfromOccupancyGrid(EnvironmentNAVXYTHETALAT& env, nav_msgs::OccupancyGrid& occupancy_grid,char* motPrimFilename) ;
  void planxythetalat(EnvironmentNAVXYTHETALAT& env,gtsam::Pose2& start,gtsam::Pose2& goal,std::vector<sbpl_xy_theta_pt_t> &xythetaPath,std::vector<int>& solution_stateIDs,bool initialized);
  void initPlanner(std::shared_ptr<SBPLPlanner>& planner,EnvironmentNAVXYTHETALAT& env) ;


  //All Navigation stuff
  void setCostMapfromOccGrid(nav_msgs::OccupancyGrid& occupancy_grid,costmap_2d::Costmap2DROS& costmap2dros);

  void fromSPBLtoROSpath( std::vector<sbpl_xy_theta_pt_t> &xythetaPath, std::vector< geometry_msgs::PoseStamped > &plan);


  void driveRobot(rosTrajectory& trajectory);


  //All aslam stuff
  double alpha_;

  void mainAslamAlgorithm();

  void getFrontierCells(nav_msgs::OccupancyGrid& occupancy_grid,vectorOfIndices& frontier_indices);
  void findFrontierClusters(vectorOfIndices& frontier_indices,vectorOfIndices& cluster_centers);


  double utilityOfTrajectory(mapping::ProbabilityMap& probability_map, spblTrajectory &trajectory);
  void selectTrajectory(mapping::ProbabilityMap& probability_map, spblTrajectoryList &trajectoryList,spblTrajectory& best_trajectory);



  //All sensor specific stuff
  void predictedMeasurement(mapping::ProbabilityMap& probability_map,spblTrajectory& trajectory,LaserScanList& predicted_scans);
  double shannonEntropy(const mapping::ProbabilityMap& probability_map,spblTrajectory& trajectory,LaserScanList& predicted_scans);
  double renyiEntopy(mapping::ProbabilityMap& probability_map,spblTrajectory& trajectory,LaserScanList& predicted_scans);

  void getProbabilityMaps(const mapping::ProbabilityMap& probabilityMap,spblTrajectory& trajectory,LaserScanList& measurements,ProbabilityMaps& probability_maps);

  //Visualization stuff
  void MarkerConfig(visualization_msgs::Marker& marker,gtsam::Pose2& pose2,int& id);
  void addToMarkerArray(visualization_msgs::MarkerArray& marker_array,gtsam::Pose2& pose2);
  void plotsbplTrajectory(spblTrajectory& trajectory);

  ~AslamBase();
};

};

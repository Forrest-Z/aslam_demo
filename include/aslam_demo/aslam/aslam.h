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


#include <laser_geometry/laser_geometry.h>

#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

#include <kdtree++/kdtree.hpp>


namespace aslam {

class AslamBase {
public:
  AslamBase();
  void getFrontierCells(nav_msgs::OccupancyGrid& ,std::vector<std::pair<int,int> >& );
  void findFrontierClusters(std::vector<std::pair<int,int> >& );
  ~AslamBase();
};

};

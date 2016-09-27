#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <aslam_demo/mapping/map_processing.h>
#include <aslam_demo/mapping/probability_map.h>

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

#include <thread>

#ifndef ASLAM_DEMO
#define ASLAM_DEMO

namespace aslam_demo
{
class AslamDemo {
private:
	ros::NodeHandle n_;

	laser_geometry::LaserProjection laser_projection_;
	tf::TransformListener tf_listener_;
	tf::TransformBroadcaster tf_broadcaster_;

	sensor_msgs::LaserScan laser_scan_;
	geometry_msgs::Twist robot_command_;
	nav_msgs::Odometry odometry_;
	nav_msgs::OccupancyGrid current_map_;

	ros::Publisher map_pub_;
	ros::Publisher pose_pub_;
	ros::Publisher command_pub_;

	ros::Subscriber<nav_msgs::Odometry> odometry_sub_;
	ros::Subscriber<sensor_msgs::LaserScan>  laser_sub_;

public:
	AslamDemo(ros::NodeHandle);
	std::atomic<bool> isactive_laser_factor_thread_,isactive_slam_thread_,isactive_navigation_thread_;

	void scanCallback (const sensor_msgs::LaserScan::ConstPtr&);
	void odomCallback (const nav_msgs::Odometry::ConstPtr&);

	void createLaserFactors();
	void slamHandler();
	void navigationHandler();

	std::thread laser_factor_thread_;
	std::thread slam_thread_;
	std::thread navigation_thread_;


};
}
#endif

#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>


#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <aslam_demo/mapping/map_processing.h>
#include <aslam_demo/mapping/probability_map.h>
#include <aslam_demo/factors/key_generator.h>
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

#include <laser_geometry/laser_geometry.h>

#include <thread>
#include <atomic>

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
    mapping::ProbabilityMap prob_map_;
    gtsam::Pose3 base_T_laser_;


	ros::Publisher map_pub_;
	ros::Publisher pose_pub_;
	ros::Publisher command_pub_;

	ros::Subscriber odometry_sub_;
	ros::Subscriber  laser_sub_;

    mapping::LaserScans laserscans_;
    std::map<ros::Time,nav_msgs::Odometry> odomreadings_;
    mapping::RelativePoseEstimates laser_poses_;
    factors::KeyGenerator key_generator_;

    gtsam::NonlinearFactorGraph factor_graph_;
    gtsam::Values initial_guess_,pose_estimates_; //@todo:initial_guess
    mapping::optimization::Covariances pose_with_cov_;
    gtsam::LevenbergMarquardtParams parameters_; //@todo:parameters

    gtsam::Pose2 getRelativeOdom(nav_msgs::Odometry &,nav_msgs::Odometry &);
    nav_msgs::Odometry getCorrespondingOdom(const ros::Time &);
    nav_msgs::OccupancyGrid fromGtsamMatrixToROS(gtsam::Matrix &);
    void fromTftoGtsamPose(gtsam::Pose3 &, const tf::Transform &);
    void createZeroInitialGuess();

public:
	AslamDemo(ros::NodeHandle);
	std::atomic<bool> isactive_laser_factor_thread_,isactive_slam_thread_,isactive_navigation_thread_;

	void scanCallback (const sensor_msgs::LaserScan::ConstPtr&);
	void odomCallback (const nav_msgs::Odometry::ConstPtr&);

	void createLaserFactors();
	void slam();
	void navigationHandler();

	std::thread laser_factor_thread_;
	std::thread slam_thread_;
	std::thread navigation_thread_;


};
}
#endif

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

#include <aslam_demo/aslam/aslam.h>



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

#ifndef ASLAM_DEMO
#define ASLAM_DEMO

namespace aslam_demo
{



class AslamDemo {
private:

	struct poseNode {

	  typedef double value_type;

	  poseNode() {

	  }
	  poseNode(value_type a, value_type b, value_type c,gtsam::Key key) {
	    d[0] = a;
	    d[1] = b;
	    d[2] = c;
	    key_ = key;
	  }

	  poseNode(const poseNode & x) {
	    d[0] = x.d[0];
	    d[1] = x.d[1];
	    d[2] = x.d[2];
	    key_ = x.key_;
	  }

	  ~poseNode() {
	  }

	  double distance_to(poseNode const& x) const {
	     double dist = 0;
	     for (int i = 0; i != 2; ++i)
	        dist += (d[i]-x.d[i])*(d[i]-x.d[i]);
	     return std::sqrt(dist);
	  }

	  double angleDistance(poseNode const& x) const {
	    return((M_PI - fabs(fabs(d[3] - x.d[3]))) - M_PI);
	  }

	  void operator=(poseNode const& A) {
	    d[0] = A.d[0];
	    d[1] = A.d[1];
	    d[2] = A.d[2];
	    key_ = A.key_;
	  }

	  inline value_type operator[](size_t const N) const { return d[N]; }
    value_type d[3];
	  gtsam::Key key_;

	};


	static inline double tac( poseNode t, size_t k ) { return t[k]; }
	double angledist(double a1,double a2) { return(M_PI - fabs(fabs(a1 - a2)) - M_PI); }
	typedef KDTree::KDTree<2, poseNode, std::pointer_to_binary_function<poseNode,size_t,double> > tree_type;
  tree_type pose_tree_;
  void updateKDTree(const gtsam::Values& );
  void searchForLoopClosure(gtsam::NonlinearFactorGraph& ,gtsam::Values& );
  void doScanMatch(sensor_msgs::LaserScan&,sensor_msgs::LaserScan&,mapping::RelativePoseEstimates& );

  std::shared_ptr<aslam::AslamBase> aslam_;
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
  gtsam::Pose2 current_pose_;


	ros::Publisher map_pub_;
	ros::Publisher pose_pub_;
	ros::Publisher command_pub_;

	ros::Subscriber odometry_sub_;
	ros::Subscriber  laser_sub_;
	ros::Subscriber  gazebo_model_state_sub_;

	ros::ServiceClient model_state_client_;
	std::map<ros::Time,gazebo_msgs::ModelState> model_state_list_;


	ros::ServiceClient map_service_client_;
	nav_msgs::GetMap srv_map_;
	gazebo_msgs::GetModelState model_state_srv_;

  mapping::LaserScans laserscans_;
  mapping::Odometry odomreadings_;
  mapping::Odometry trueodomreadings_;

  mapping::RelativePoseEstimates laser_poses_;
  mapping::RelativePoseEstimates laser_pose_cache_;

  factors::KeyGenerator key_generator_;

  const double time_tolerance;
  bool map_initialized_ = false;

  gtsam::NonlinearFactorGraph factor_graph_;
  gtsam::Values initial_guess_,pose_estimates_; //@todo:initial_guess
  mapping::optimization::Covariances pose_with_cov_;
  gtsam::LevenbergMarquardtParams parameters_; //@todo:parameters

  gtsam::Pose2 getRelativeOdom(nav_msgs::Odometry &,nav_msgs::Odometry &);
  nav_msgs::Odometry getCorrespondingOdom(const ros::Time &,mapping::Odometry&);
  nav_msgs::OccupancyGrid fromGtsamMatrixToROS(gtsam::Matrix &);
  void fromTftoGtsamPose(gtsam::Pose3 &, const tf::Transform &);
  void fromGtsamPose2toTf(const gtsam::Pose2 &, tf::Transform &);

  void createZeroInitialGuess();
  void connectWithOdometry(gtsam::NonlinearFactorGraph&,gtsam::Values&);
  gtsam::Pose2 extractLatestPose(const gtsam::Values&);

  void getTrueEstimates(gtsam::Values& ,gtsam::Values& );
  void FromQuaternionToRPY(tf::Quaternion& ,double& ,double&, double&);

public:
	AslamDemo(ros::NodeHandle&);
	~AslamDemo();
	std::atomic<bool> isactive_laser_factor_thread_,isactive_slam_thread_,isactive_navigation_thread_;

	void scanCallback (const sensor_msgs::LaserScan::ConstPtr&);
	void odomCallback (const nav_msgs::Odometry::ConstPtr&);
	void slam();
	void getMapCallback (nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res);
	void gazeboModelStateCallback(const gazebo_msgs::ModelStates::ConstPtr&);

	void createLaserFactors();
	void slamHandler();
	void navigationHandler();
	void doAslamStuff(mapping::ProbabilityMap& map);
	void tfInit();


	std::mutex slam_mutex;
	std::condition_variable slam_cv;


	std::thread laser_factor_thread_;
	std::thread slam_thread_;
	std::thread navigation_thread_;
	std::shared_ptr<std::thread> tf_init_thread_;


};
}
#endif

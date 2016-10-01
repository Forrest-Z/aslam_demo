#include "aslam_demo/aslam_demo.h"

namespace aslam_demo {

AslamDemo::AslamDemo(ros::NodeHandle n):n_(n) {
	laser_sub_ = n_.subscribe<sensor_msgs::LaserScan>("/scan",1000,boost::bind(&AslamDemo::scanCallback,this,_1));
	odometry_sub_ = n_.subscribe<nav_msgs::Odometry>("/odom",1000,boost::bind(&AslamDemo::odomCallback,this,_1));
	tf::StampedTransform stamped_transform;
	tf::Transform transform;
	try {
		tf_listener_.lookupTransform("/base_link", "/laser_link",
	                               ros::Time(0), stamped_transform);
		transform = stamped_transform;
		fromTftoGtsamPose(base_T_laser_,transform);
	}
	catch (tf::TransformException &ex) {
	      ROS_ERROR("%s",ex.what());
	      ros::Duration(1.0).sleep();
    }

	//map_pub_ = n_.advertise<nav_msgs::OccupancyGrid>("curr_map",1);
	pose_pub_ = n_.advertise<geometry_msgs::Pose2D>("curr_pose",1);
	command_pub_ = n_.advertise<geometry_msgs::Twist>("command",1);
    //slam_thread_ = std::thread(slam);

}

void AslamDemo::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_ptr) {
	if (laserscans_.empty()) {
		laserscans_[scan_ptr->header.stamp] = *scan_ptr;
		return;
	}
	ROS_INFO_STREAM("A");
	sensor_msgs::LaserScan latest_scan = laserscans_.rbegin()->second;
	struct sm_params csm_params;
	ROS_INFO_STREAM("A1");

    //Enter the transform form base_link to laser_link
    nav_msgs::Odometry odom1 = getCorrespondingOdom(latest_scan.header.stamp);
    nav_msgs::Odometry odom2 = getCorrespondingOdom(scan_ptr->header.stamp);
	ROS_INFO_STREAM("BB");

    gtsam::Pose2 initial_pose = getRelativeOdom(odom1,odom2);
//	ROS_INFO_STREAM("B"<<latest_scan<<*scan_ptr);
    initial_pose.print();
//    ROS_INFO_STREAM("C22");
    base_T_laser_.print();
    mapping::RelativePoseEstimate laser_pose;
    try {
    	laser_pose = mapping::csm::computeLaserScanMatch(latest_scan,
    		*scan_ptr,
			csm_params,
			initial_pose,base_T_laser_,.05,100000000000000,1000000000000000,"../");
        laser_poses_.push_back(laser_pose);

    }
    catch(std::exception &ex) {
    	ROS_ERROR("%s",ex.what());
        ROS_INFO_STREAM("EEE");
  //  	return;
    }

	ROS_INFO_STREAM("D");

	ROS_INFO_STREAM("C");

	laserscans_[scan_ptr->header.stamp] = *scan_ptr;
}

void AslamDemo::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_ptr) {
	if (odomreadings_.empty()) {
		odomreadings_[odom_ptr->header.stamp] = *odom_ptr;
		return;
	}
	odomreadings_[odom_ptr->header.stamp] = *odom_ptr;
    slam();
}

void AslamDemo::createZeroInitialGuess() {
	initial_guess_.clear();
	gtsam::KeySet keys = factor_graph_.keys();
	for (auto const iter: keys) {
		initial_guess_.insert(iter,gtsam::Pose2(0.0,0.0,0.0));
	}
}

void AslamDemo::slam() {
	if (laser_poses_.empty())
	{
		return;
	}
	factor_graph_ = mapping::laserscan::createLaserScanFactors(laser_poses_,1.e-09);
	createZeroInitialGuess();

	if(!mapping::optimization::validateFactorGraph(factor_graph_,initial_guess_)) {
		ROS_INFO_STREAM("Not Validated!!");
		return;
	}
	pose_estimates_ = mapping::optimization::optimizeFactorGraph(factor_graph_,initial_guess_,parameters_);
	if (pose_estimates_.size() < 100) {
		return;
	}
	//pose_with_cov_ = mapping::optimization::computeCovariances(factor_graph_,pose_estimates_);
	mapping::map::buildMap(prob_map_,pose_estimates_,laserscans_,base_T_laser_,.001,1.e-09,"");
	prob_map_.occupancyGrid("../currmap");
	ROS_INFO_STREAM("Map Formed!!");

//	current_map_  = fromGtsamMatrixToROS(occupancy_map);
//	map_pub_.publish(current_map_);
}

gtsam::Pose2 AslamDemo::getRelativeOdom(nav_msgs::Odometry &odom1,nav_msgs::Odometry &odom2) {
	gtsam::Pose2 pose;
	ROS_INFO_STREAM("AA");

	pose = mapping::odometry::splitOdometry(odom1,odom2,odom1.header.stamp,odom2.header.stamp);
	return pose;
}

nav_msgs::Odometry AslamDemo::getCorrespondingOdom(const ros::Time &time_stamp) {
	nav_msgs::Odometry odom;
	ROS_INFO_STREAM("B1");
	while(odomreadings_.empty());
	auto iter = odomreadings_.upper_bound(time_stamp);
	if (iter != odomreadings_.end()) {
		odom = iter->second;
	}
	else {
		ROS_INFO_STREAM("B3");
		//auto iter = (odomreadings_.lower_bound(time_stamp));
	//	if (iter == odomreadings_.end()) ROS_INFO_STREAM("WTF");
		odom = std::prev(iter,1)->second;
		ROS_INFO_STREAM("B3");

	}
	ROS_INFO_STREAM(odomreadings_.size());
	return odom;
}

void AslamDemo::fromTftoGtsamPose(gtsam::Pose3 &pose3, const tf::Transform &transform) {
	tf::Vector3 translation = transform.getOrigin();
	tf::Matrix3x3 rotation = transform.getBasis();
	gtsam::Point3 trans(translation.getX(),translation.getY(),translation.getZ());
	gtsam::Rot3 rot(rotation[0][0],rotation[0][1],rotation[0][2],rotation[1][0],rotation[1][1],rotation[1][2],rotation[2][0],rotation[2][1],rotation[2][2]);
	gtsam::Pose3 new_pose(rot,trans);
	pose3 = new_pose;

}

}

int main(int argc, char *argv[]) {

	ros::init(argc,argv,"aslam_demo_node");
	ros::NodeHandle n;
	aslam_demo::AslamDemo main_obj(n);
	ros::spin();
	return 0;

}


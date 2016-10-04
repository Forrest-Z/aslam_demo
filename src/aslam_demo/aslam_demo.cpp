#include "aslam_demo/aslam_demo.h"

namespace aslam_demo {

AslamDemo::AslamDemo(ros::NodeHandle n):n_(n),key_generator_(factors::KeyGenerator(AslamDemo::time_tolerance)),isactive_slam_thread_(true) {
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
	transform.setIdentity();
	//map_pub_ = n_.advertise<nav_msgs::OccupancyGrid>("curr_map",1);
	pose_pub_ = n_.advertise<geometry_msgs::Pose2D>("curr_pose",1);
	command_pub_ = n_.advertise<geometry_msgs::Twist>("command",1);
   // slam_thread_ = std::thread(&AslamDemo::slamHandler,this);

}

void AslamDemo::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_ptr) {
	if (laserscans_.empty()) {
		laserscans_[scan_ptr->header.stamp] = *scan_ptr;
		return;
	}
	sensor_msgs::LaserScan latest_scan = laserscans_.rbegin()->second;
	struct sm_params csm_params;

    //Enter the transform form base_link to laser_link
    nav_msgs::Odometry odom1 = getCorrespondingOdom(latest_scan.header.stamp);
    nav_msgs::Odometry odom2 = getCorrespondingOdom(scan_ptr->header.stamp);

    gtsam::Pose2 initial_pose = getRelativeOdom(odom1,odom2);
    if (std::isnan(initial_pose.x()) || std::isnan(initial_pose.x()) || std::isnan(initial_pose.x())) {
    	initial_pose =  gtsam::Pose2(0.0,0.0,0.0);
    }
    mapping::RelativePoseEstimate laser_pose;
    try {
    	laser_pose = mapping::csm::computeLaserScanMatch(latest_scan,
    		*scan_ptr,
			csm_params,
			initial_pose,base_T_laser_,.05,100000000000000,1000000000000000,"../");
    	laser_pose.relative_pose.print("Laser Scan Match:");
        laser_poses_.push_back(laser_pose);

    }
    catch(std::exception &ex) {
    	ROS_ERROR("%s",ex.what());
  //  	return;
    }

    if (laser_poses_.size()%100 == 0) {
    	slam();
    }

	laserscans_[scan_ptr->header.stamp] = *scan_ptr;

}


void AslamDemo::slamHandler() {
	std::unique_lock<std::mutex> lk(slam_mutex);
	while(isactive_slam_thread_) {
		slam_cv.wait(lk,[&](){return (laser_poses_.size() % 200 == 0 && !isactive_slam_thread_);});
		if (!isactive_slam_thread_) {
			return;
		}
		slam();
	}
}

void AslamDemo::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_ptr) {
	nav_msgs::Odometry odom = *odom_ptr;
	if (odomreadings_.empty()) {
		odomreadings_[odom.header.stamp] = odom;
		return;
	}
	odomreadings_[odom.header.stamp] = odom;
}

void AslamDemo::createZeroInitialGuess() {
	initial_guess_.clear();
	gtsam::KeySet keys = factor_graph_.keys();
	for (auto const iter: keys) {
		initial_guess_.insert(iter,gtsam::Pose2(0.0,0.0,0.0));
	}
}


void AslamDemo::connectWithOdometry(gtsam::NonlinearFactorGraph& factor_graph) {
    ROS_INFO_STREAM("Connect with odom entered");
	initial_guess_.clear();

	gtsam::KeySet keys = factor_graph.keys();
	mapping::Timestamps timestamps;
	gtsam::Vector sigmas(6);

	sigmas<<0.01,0.01,0.01,0.01,0.01,0.01;

	for (auto const iter: keys) {
//		ROS_INFO_STREAM("Keys:"<<iter);
		timestamps.insert(key_generator_.extractTimestamp(iter));
//		initial_guess_.insert(iter,gtsam::Pose2(0.0,0.0,0.0));

	}
	mapping::RelativePoseEstimates relative_estimates = mapping::odometry::computeRelativePoses(odomreadings_,timestamps,sigmas,time_tolerance);
	gtsam::NonlinearFactorGraph odom_graph = mapping::odometry::createOdometryFactors(relative_estimates,time_tolerance,keys);

	gtsam::Pose2 curr_pose = gtsam::Pose2(0.0,0.0,0.0);

	auto ptr = keys.begin();
	initial_guess_.insert(*ptr,curr_pose);

	for (auto const iter: relative_estimates) {
		curr_pose = curr_pose.compose(iter.relative_pose);

	//	iter.relative_pose.print();
	//	key = key_generator_.generateKey(factors::key_type::Pose2,iter.timestamp2);
		ptr = std::next(ptr,1);

		initial_guess_.insert(*ptr,curr_pose);
	//	ROS_INFO_STREAM("Next Pose"<<key);
	}

	factor_graph.push_back(odom_graph);

}
gtsam::Pose2 AslamDemo::extractLatestPose(const gtsam::Values& values) {
	return(values.at<gtsam::Pose2>(*(values.keys().rbegin())));
}

void AslamDemo::fromGtsamPose2toTf(const gtsam::Pose2 &pose2, tf::Transform &transform) {
	tf::Vector3 translation(pose2.x(),pose2.y(),0);
	tf::Matrix3x3 rotation;
	rotation.setRPY(0.0,0.0,pose2.theta());
	transform.setOrigin(translation);
	transform.setBasis(rotation);
}

void AslamDemo::slam() {
	if (laser_poses_.empty()) {
		return;
	}
	factor_graph_ = mapping::laserscan::createLaserScanFactors(laser_poses_,time_tolerance);
	connectWithOdometry(factor_graph_);
//	initial_guess_.print();
//	createZeroInitialGuess();

	if(!mapping::optimization::validateFactorGraph(factor_graph_,initial_guess_)) {
		ROS_INFO_STREAM("Not Validated!!");
		return;
	}
	pose_estimates_ = mapping::optimization::optimizeFactorGraph(factor_graph_,initial_guess_,parameters_);
	gtsam::Pose2 current_pose = extractLatestPose(pose_estimates_);
	current_pose.print("Current Pose: ");

	if (!pose_estimates_.size()) {
		return;
	}
	//pose_with_cov_ = mapping::optimization::computeCovariances(factor_graph_,pose_estimates_);

//	mapping::map::buildMap(prob_map_,pose_estimates_,laserscans_,base_T_laser_,.001,time_tolerance,"map");
	pose_estimates_.clear();
	mapping::LaserScans singlescan;
	singlescan[laserscans_.rbegin()->first-ros::Duration(2*time_tolerance)] = laserscans_.rbegin()->second;
	singlescan[laserscans_.rbegin()->first-ros::Duration(time_tolerance)] = laserscans_.rbegin()->second;

	singlescan[laserscans_.rbegin()->first] = laserscans_.rbegin()->second;
	singlescan[laserscans_.rbegin()->first+ros::Duration(time_tolerance)] = laserscans_.rbegin()->second;
	singlescan[laserscans_.rbegin()->first+ros::Duration(2*time_tolerance)] = laserscans_.rbegin()->second;

	ROS_INFO_STREAM(singlescan.lower_bound(laserscans_.rbegin()->first-ros::Duration(time_tolerance))->first<<singlescan.upper_bound(laserscans_.rbegin()->first+ros::Duration(time_tolerance))->first);
	pose_estimates_.insert(key_generator_.generateKey(factors::key_type::Pose2,laserscans_.rbegin()->first),gtsam::Pose2(0.1,0.1,0.3));
	//pose_estimates_.insert(key_generator_.generateKey(factors::key_type::Pose2,std::next(singlescan.begin(),1)->first),gtsam::Pose2(0.1,0.1,0.3));

	pose_estimates_.print("Pose Estimates");
	if (!map_initialized_) {
			prob_map_ = mapping::map::createEmptyMap(pose_estimates_,.02,10);
			map_initialized_ = false;
	}
	ROS_INFO_STREAM("Map Initialized");
	mapping::map::buildMap(prob_map_,pose_estimates_,singlescan,base_T_laser_,.001,time_tolerance,"map");
	ROS_INFO_STREAM("Map Formed!!");

	current_pose = gtsam::Pose2(0.1,0.1,0.3);
	std::string filename = "currmap";
	prob_map_.occupancyGrid(filename);

	tf::Transform transform;
	fromGtsamPose2toTf(current_pose,transform);
	tf_broadcaster_.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), "base_link","map" ));


//	current_map_  = fromGtsamMatrixToROS(occupancy_map);
//	map_pub_.publish(current_map_);
}

gtsam::Pose2 AslamDemo::getRelativeOdom(nav_msgs::Odometry &odom1,nav_msgs::Odometry &odom2) {
	gtsam::Pose2 pose;

	pose = mapping::odometry::splitOdometry(odom1,odom2,odom1.header.stamp,odom2.header.stamp);
	return pose;
}

nav_msgs::Odometry AslamDemo::getCorrespondingOdom(const ros::Time &time_stamp) {
	nav_msgs::Odometry odom;
	while(odomreadings_.empty());
	auto iter = odomreadings_.upper_bound(time_stamp);
	if (iter != odomreadings_.end()) {
		odom = iter->second;
	}
	else {
		odom = std::prev(iter,1)->second;
	}
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


AslamDemo::~AslamDemo() {
//	isactive_slam_thread_ = false;
//	if(slam_thread_.joinable()) slam_thread_.join();
}
}

int main(int argc, char *argv[]) {

	ros::init(argc,argv,"aslam_demo_node");
	ros::NodeHandle n;
	aslam_demo::AslamDemo main_obj(n);
	ros::spin();
	return 0;

}


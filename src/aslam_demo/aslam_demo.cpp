#include "aslam_demo/aslam_demo.h"

namespace aslam_demo {

AslamDemo::AslamDemo(ros::NodeHandle& n):n_(n),time_tolerance(0.0001),key_generator_(time_tolerance),isactive_slam_thread_(true) {
  ROS_INFO_STREAM("A");

  ROS_INFO_STREAM("Transform Sent");
  tf_init_thread_ = std::make_shared<std::thread>(boost::bind(&AslamDemo::tfInit,this));
  aslam_ = std::make_shared<aslam::AslamBase>(n);

	laser_sub_ = n_.subscribe<sensor_msgs::LaserScan>("/scan",1000000,boost::bind(&AslamDemo::scanCallback,this,_1));
	odometry_sub_ = n_.subscribe<nav_msgs::Odometry>("/odom",1000,boost::bind(&AslamDemo::odomCallback,this,_1));
	gazebo_model_state_sub_ = n_.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states",1000,boost::bind(&AslamDemo::gazeboModelStateCallback,this,_1));
///	f = std::bind(&AslamDemo::tac, this, _1, _2);
 	pose_tree_ = tree_type(std::ptr_fun(tac));
	ROS_INFO_STREAM("A");
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
	ROS_INFO_STREAM("B");

	current_pose_ = gtsam::Pose2(0.0,0.0,0.0);
	transform.setIdentity();
	map_pub_ = n_.advertise<nav_msgs::OccupancyGrid>("map",1);
	pose_pub_ = n_.advertise<geometry_msgs::Pose2D>("curr_pose",1);
	command_pub_ = n_.advertise<geometry_msgs::Twist>("command",1);
	ROS_INFO_STREAM("C");

  map_service_client_ = n_.serviceClient<nav_msgs::GetMap>("static_map");
  model_state_client_ = n_.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
	ROS_INFO_STREAM("D");

   // slam_thread_ = std::thread(&AslamDemo::slamHandler,this);
}

void AslamDemo::getMapCallback (nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res) {

}

void AslamDemo::tfInit() {
  int wait = 100000;
    while(--wait) {
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf::Transform::getIdentity(), ros::Time::now(), "/base_link","/map" ));
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf::Transform::getIdentity(), ros::Time::now()+ros::Duration(5.0), "/base_link","/map" ));

   }

}


void AslamDemo::gazeboModelStateCallback(const gazebo_msgs::ModelStates::ConstPtr& input) {
	for(size_t i = 0;i < input->name.size();i++) {
		if(input->name[i] == "mobile_base") {
		  nav_msgs::Odometry odom_temp;
		  odom_temp.header.frame_id = "base_footprint";
		  odom_temp.header.stamp = ros::Time::now();
		  odom_temp.pose.pose = input->pose[i];
		  odom_temp.pose.pose.position.z = 0.0;
		  odom_temp.twist.twist = input->twist[i];
		  trueodomreadings_[ros::Time::now()] = odom_temp;
		}
	}
}


void AslamDemo::doScanMatch(sensor_msgs::LaserScan& latest_scan,sensor_msgs::LaserScan& current_scan,mapping::RelativePoseEstimates& relative_poses) {
  struct sm_params csm_params;

      //Enter the transform form base_link to laser_link
  nav_msgs::Odometry odom1 = getCorrespondingOdom(latest_scan.header.stamp,odomreadings_);
  nav_msgs::Odometry odom2 = getCorrespondingOdom(current_scan.header.stamp,odomreadings_);

  gtsam::Pose2 initial_pose = getRelativeOdom(odom1,odom2);
  if (std::isnan(initial_pose.x()) || std::isnan(initial_pose.x()) || std::isnan(initial_pose.x())) {
    initial_pose =  gtsam::Pose2(0.0,0.0,0.0);
  }
  mapping::RelativePoseEstimate laser_pose;
  try {
    laser_pose = mapping::csm::computeLaserScanMatch(latest_scan,
        current_scan,
    csm_params,
    initial_pose,base_T_laser_,.1,100000000000000,1000000000000000,"../");
    laser_pose.relative_pose.print("Laser Scan Match:");
     // laser_poses_.push_back(laser_pose);
    relative_poses.push_back(laser_pose);
  }
  catch(std::exception &ex) {
    ROS_ERROR("%s",ex.what());
    return;
  }

}

void AslamDemo::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_ptr) {
	if (laserscans_.empty()) {
		laserscans_[scan_ptr->header.stamp] = *scan_ptr;
		return;
	}
	sensor_msgs::LaserScan latest_scan = laserscans_.rbegin()->second;
	sensor_msgs::LaserScan current_scan = *scan_ptr;
	doScanMatch(latest_scan,current_scan,laser_pose_cache_);

    if (laser_pose_cache_.size()%10 == 0) {
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

	if(!trueodomreadings_.empty()) {
//ROS_INFO_STREAM("Odom Stream:"<<odom.pose.pose.position<<"\t"<<getCorrespondingOdom(odom.header.stamp,trueodomreadings_).pose.pose.position);
	}

}

void AslamDemo::createZeroInitialGuess() {
	initial_guess_.clear();
	gtsam::KeySet keys = factor_graph_.keys();
	for (auto const iter: keys) {
		initial_guess_.insert(iter,gtsam::Pose2(0.0,0.0,0.0));
	}
}


void AslamDemo::connectWithOdometry(gtsam::NonlinearFactorGraph& factor_graph,gtsam::Values& initial_guess) {

  ROS_INFO_STREAM("Connect with odom entered");
	gtsam::KeySet keys = factor_graph.keys();
	mapping::Timestamps extracted_timestamps,timestamps;
	gtsam::Vector sigmas(6);
	double time_threshold = 5.0;
	ros::Time curr_time,prev_time;
	ros::Duration delta_time;
	for(auto const iter:keys) {
	  ros::Time timestamp = key_generator_.extractTimestamp(iter);
	  if(key_generator_.generateKey(factors::key_type::Pose2,timestamp) == iter) ROS_INFO_STREAM("I'm not insane");
	  extracted_timestamps.insert(timestamp);
	}
	sigmas<<0.1,0.1,0.1,0.1,0.1,0.1;
	bool is_first = true;
	for (auto const curr_time: extracted_timestamps) {
//		ROS_INFO_STREAM("Keys:"<<iter);
	 // curr_time = key_generator_.extractTimestamp(iter);

	  if(!is_first) {
	    delta_time = curr_time - prev_time;
	    ros::Time new_time = prev_time;

	    if(delta_time.toSec() > time_threshold) {
	      while(new_time < curr_time) {

	        ROS_INFO_STREAM("New Time"<<new_time<<"Delta_time"<<delta_time.toSec());
          new_time += ros::Duration(1.0);
          gtsam::Key new_key = key_generator_.generateKey(factors::key_type::Pose2,new_time);
          keys.insert(new_key);
          timestamps.insert(new_time);

	      }
	    }
	  }
	  is_first = false;
		timestamps.insert(curr_time);
		prev_time = curr_time;
//		initial_guess_.insert(iter,gtsam::Pose2(0.0,0.0,0.0));
	}
	if(!timestamps.size()) return;
  for(auto const iter: timestamps) ROS_INFO_STREAM("Timestamp"<<iter);

	mapping::RelativePoseEstimates relative_estimates = mapping::odometry::computeRelativePoses(odomreadings_,timestamps,sigmas,time_tolerance);
  for(auto const iter: relative_estimates) iter.relative_pose.print("Rel P");

	gtsam::NonlinearFactorGraph odom_graph = mapping::odometry::createOdometryFactors(relative_estimates,time_tolerance,keys);

	gtsam::Pose2 curr_pose = current_pose_;
	odom_graph.print("Odom Graph");
	auto ptr = keys.begin();
	initial_guess.insert(*ptr,curr_pose);

	for (auto const iter: relative_estimates) {
		curr_pose = curr_pose.compose(iter.relative_pose);
		ptr = std::next(ptr,1);
		initial_guess.insert(*ptr,curr_pose);
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

void AslamDemo::updateKDTree(const gtsam::Values& values) {
	for(auto const iter: values) {
		gtsam::Pose2 pose = values.at<gtsam::Pose2>(iter.key);
		poseNode node(pose.x(),pose.y(),pose.theta(),iter.key);
		pose_tree_.insert(node);
	}
}

void AslamDemo::searchForLoopClosure(gtsam::NonlinearFactorGraph& factor_graph,gtsam::Values& values) {
  ROS_INFO_STREAM("Search for loop closure"<<factor_graph.size()<<"\t"<<values.size());
  if(factor_graph.size()) return;

  for(auto const iter: values) {
    gtsam::Pose2 pose = values.at<gtsam::Pose2>(iter.key);
    poseNode input(pose.x(),pose.y(),pose.theta(),iter.key);
    std::vector<poseNode> neighbours;
    double limit = 1.0;
    pose_tree_.find_within_range(input,limit,std::back_insert_iterator<std::vector<poseNode> >(neighbours));
    if (!neighbours.size()) continue;
//    std::pair<tree_type::const_iterator,double> found = pose_tree_.find_nearest(input);
    ros::Time input_time = key_generator_.extractTimestamp(input.key_);
    poseNode maxNode;
    ros::Time max_node_stamp;
    double angle_threshold = .5,time_threshold = 10.0,min_angle_dist = 100.0;
    for(auto const &iter: neighbours) {
      ros::Time neighbour_time = key_generator_.extractTimestamp(iter.key_);
      double angle_dist = input.angleDistance(iter);
      if((input_time - neighbour_time).toSec() > time_threshold && angle_dist <  angle_threshold) {
        if(angle_dist < min_angle_dist) {
          max_node_stamp = neighbour_time;
          min_angle_dist = angle_dist;
          maxNode = iter;
        }
      }
    }
    if(min_angle_dist == 100.0) continue;
    auto iter1 = laserscans_.lower_bound(input_time);
    auto iter2 = laserscans_.lower_bound(max_node_stamp);

    if(iter1 != laserscans_.end() && iter2 != laserscans_.end()) {

      mapping::RelativePoseEstimates relative_poses;
      ROS_INFO_STREAM("Loop Closure Matching");
      doScanMatch(iter1->second,iter2->second,relative_poses);
      ROS_INFO_STREAM("Loop Closure Matching Done");

      if(relative_poses.size() != 0) {
        const gtsam::Pose2 relative_pose = relative_poses.begin()->relative_pose;
        gtsam::noiseModel::Base::shared_ptr noise_model(gtsam::noiseModel::Gaussian::Covariance(relative_poses.begin()->cov, true));
        gtsam::NonlinearFactor::shared_ptr factor(new factors::LaserScanFactor(input.key_, maxNode.key_, relative_pose, noise_model));
        factor_graph.push_back(factor);
      }
    }

   /* poseNode output;
    ros::Time timestamp1,timestamp2;
    double time_threshold = 10.0;
    if(found.first != pose_tree_.end()) {

      output = *(found.first);
      timestamp1 = key_generator_.extractTimestamp(input.key_);
      timestamp2 = key_generator_.extractTimestamp(output.key_);

      if((timestamp1 - timestamp2).toSec() > time_threshold) {

        auto iter1 = laserscans_.lower_bound(timestamp1);
        auto iter2 = laserscans_.lower_bound(timestamp2);

        if(iter1 != laserscans_.end() && iter2!=laserscans_.end()){

          mapping::RelativePoseEstimates relative_poses;
          ROS_INFO_STREAM("Loop Closure Matching");
          doScanMatch(iter1->second,iter2->second,relative_poses);

          if(relative_poses.size() != 0) {
            const gtsam::Pose2 relative_pose = relative_poses.begin()->relative_pose;
            gtsam::noiseModel::Base::shared_ptr noise_model(gtsam::noiseModel::Gaussian::Covariance(relative_poses.begin()->cov, true));
            gtsam::NonlinearFactor::shared_ptr factor(new factors::LaserScanFactor(input.key_, output.key_, relative_pose, noise_model));
            factor_graph.push_back(factor);
          }
        }
      }

      ROS_INFO_STREAM("Queried Pose:"<<"("<<input.d[0]<<","<<input.d[1]<<","<<input.d[2]<<")\t"<<"Returned Pose:("<<output.d[0]<<","<<output.d[1]<<","<<output.d[2]<<")"<<"T1:"<<key_generator_.extractTimestamp(input.key_)<<"\tT2:"<<key_generator_.extractTimestamp(output.key_));
    }
    else {
      ROS_INFO_STREAM("KDTree search failed!!");
    }*/
  }
}

void AslamDemo::getTrueEstimates(gtsam::Values& input_estimates,gtsam::Values& true_estimates) {
	for(auto const iter: input_estimates) {
		ros::Time timestamp = key_generator_.extractTimestamp(iter.key);
		nav_msgs::Odometry corresponding_odom = getCorrespondingOdom(timestamp,trueodomreadings_);
		tf::Quaternion q(corresponding_odom.pose.pose.orientation.x,corresponding_odom.pose.pose.orientation.y,corresponding_odom.pose.pose.orientation.z,corresponding_odom.pose.pose.orientation.w);
		double roll,pitch,yaw;
		FromQuaternionToRPY(q,roll,pitch,yaw);
		gtsam::Pose2 pose = gtsam::Pose2(corresponding_odom.pose.pose.position.x,corresponding_odom.pose.pose.position.y,yaw);
		true_estimates.insert(iter.key,pose);
	}
}

void AslamDemo::FromQuaternionToRPY(tf::Quaternion& q,double& roll,double& pitch, double& yaw) {
	 tf::Matrix3x3 m(q);
     m.getRPY(roll, pitch, yaw);
}

void AslamDemo::slam() {
	if (laser_pose_cache_.empty() ) {
		return;
	}
//	factor_graph_ = mapping::laserscan::createLaserScanFactors(laser_poses_,time_tolerance);
	gtsam::NonlinearFactorGraph factor_graph = mapping::laserscan::createLaserScanFactors(laser_pose_cache_,time_tolerance);
	gtsam::Values initial_guess,pose_estimates;
//	connectWithOdometry(factor_graph_);
	connectWithOdometry(factor_graph,initial_guess);
//	ROS_INFO_STREAM("Initial Guess"<<initial_guess_.size());
	if(!mapping::optimization::validateFactorGraph(factor_graph,initial_guess)) {
	//if(!mapping::optimization::validateFactorGraph(factor_graph_,initial_guess_)) {
		ROS_INFO_STREAM("Not Validated!!");
		return;
	}
	factor_graph.print();
	initial_guess.print();
//	pose_estimates_ = mapping::optimization::optimizeFactorGraph(factor_graph_,initial_guess_,parameters_);
	pose_estimates = mapping::optimization::optimizeFactorGraph(factor_graph,initial_guess,parameters_);
//	gtsam::Values true_estimates;
//	getTrueEstimates(pose_estimates,true_estimates);
//	pose_estimates = true_estimates;
	current_pose_ = extractLatestPose(pose_estimates);
	current_pose_.print("Current Pose: ");
	searchForLoopClosure(factor_graph_,pose_estimates);
	updateKDTree(pose_estimates);
	if (!pose_estimates.size()) {
		return;
	}
	//pose_with_cov_ = mapping::optimization::computeCovariances(factor_graph_,pose_estimates_);


//	pose_estimates_.print("Pose Estimates");
//	prob_map_.clear();
	if (!map_initialized_) {
			prob_map_ = mapping::map::createEmptyMap(pose_estimates,.025,15.0);
			map_initialized_ = true;
	}

	mapping::map::buildMap(prob_map_,pose_estimates,laserscans_,base_T_laser_,.1,time_tolerance,"curr_map");

	ROS_INFO_STREAM("Map Initialized");
	ROS_INFO_STREAM("Map Formed!!");

	std::string filename = "currmap";

	prob_map_.occupancyGrid(current_map_);
	//prob_map_.print("Prob Map");
	tf::Transform transform;
	fromGtsamPose2toTf(current_pose_,transform);
	tf_broadcaster_.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), "base_link","map" ));

	ROS_INFO_STREAM("Map Formed"<<prob_map_.origin());
//	current_map_  = fromGtsamMatrixToROS(occupancy_map);
	map_pub_.publish(current_map_);
	doAslamStuff(prob_map_);
//	pose_estimates_.insert(pose_estimates);
	factor_graph_.push_back(factor_graph);
	laser_poses_.insert(laser_poses_.end(),laser_pose_cache_.begin(),laser_pose_cache_.end());
	laser_pose_cache_.clear();
}

gtsam::Pose2 AslamDemo::getRelativeOdom(nav_msgs::Odometry &odom1,nav_msgs::Odometry &odom2) {
	gtsam::Pose2 pose;

	pose = mapping::odometry::splitOdometry(odom1,odom2,odom1.header.stamp,odom2.header.stamp);
	return pose;
}

nav_msgs::Odometry AslamDemo::getCorrespondingOdom(const ros::Time &time_stamp,mapping::Odometry& odomreadings) {
	nav_msgs::Odometry odom;
	while(odomreadings.empty());
	auto iter = odomreadings.upper_bound(time_stamp);
	if (iter != odomreadings.end()) {
		odom = iter->second;
	}
	else {
		odom = std::prev(iter,1)->second;
	}
	return odom;
}

void AslamDemo::doAslamStuff(mapping::ProbabilityMap& map) {
  /*std::vector<std::pair<int,int> > f,g;
  aslam_->getFrontierCells(occupancy_grid,f);
  ROS_INFO_STREAM("Frontier Size"<<f.size());

  aslam_->findFrontierClusters(f,g);*/
  aslam_->updateFromProbMap(map,current_pose_);
 // while(1);
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


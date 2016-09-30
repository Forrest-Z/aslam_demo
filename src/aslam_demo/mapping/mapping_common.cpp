/**
 * mapping_common.cpp
 */

#include <aslam_demo/mapping/mapping_common.h>
#include <aslam_demo/mapping/timer.h>
#include <tf2_msgs/TFMessage.h>
#include <rosbag/view.h>
#include <ros/console.h>
#include <boost/archive/polymorphic_xml_iarchive.hpp>
#include <boost/archive/polymorphic_xml_oarchive.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include <fstream>

namespace mapping {

/* ************************************************************************* */
void printProgressBar(const std::string& title, int percent) {
  // clip percent to [0 100] range
  percent = std::max(0, std::min(100, percent));

  // Convert percent to the number of filled in characters
  int count = percent / 2;

  // Create the progress bar text
  std::string progress_bar = std::string(count, '#') + std::string(50 - count, ' ');

  // Print the message
  std::cout << "\r" << title << " [" << progress_bar << "] " << percent << "%" << std::flush;
}

/* ************************************************************************* */
void openBagfiles(const std::vector<std::string>& filenames, std::vector<rosbag::Bag>& bags) {
  // Check that at least one input bagfile was provided
  if(filenames.size() == 0) {
    throw std::runtime_error("No input bagfiles provided.");
  }

  // Resize the bags vector to the correct count
  bags.resize(filenames.size());

  // Open each bag file
  for(size_t i = 0; i < filenames.size(); ++i) {
    try{
      // Open the next bagfile
      bags[i].open(filenames[i], rosbag::bagmode::Read);
    } catch(const rosbag::BagException& e) {
      throw std::runtime_error("Unable to open bagfile '" + filenames[i] + "'. Error: " + std::string(e.what()));
    }
  }
}

/* ************************************************************************* */
void closeBagfiles(std::vector<rosbag::Bag>& bags) {
  for(size_t i = 0; i < bags.size(); ++i) {
    try{
      // Close the bagfile
      bags[i].close();
    } catch(const rosbag::BagException& e) {
      throw std::runtime_error("Unable to close bagfile '" + bags[i].getFileName() + "'. Error: " + std::string(e.what()));
    }
  }
}

/* ************************************************************************* */
boost::shared_ptr<rosbag::View> queryBagfiles(const std::vector<rosbag::Bag>& bags, const std::vector<std::string>& topics, const TimeRange& time_range) {
  boost::shared_ptr<rosbag::View> view(new rosbag::View);

  // Create the query object
  rosbag::TopicQuery query(topics);

  // Add each bagfile to the view
  for(size_t i = 0; i < bags.size(); ++i) {
    try{
      if(topics.empty()) {
        view->addQuery(bags[i], time_range.first, time_range.second);
      } else {
        view->addQuery(bags[i], query, time_range.first, time_range.second);
      }
    } catch(const rosbag::BagException& e) {
      throw std::runtime_error("Unable to add bagfile '" + bags[i].getFileName() + "' to the view. Error: " + std::string(e.what()));
    }
  }

  return view;
}

/* ************************************************************************* */
void loadStaticTf(const std::vector<rosbag::Bag>& bags, tf2_ros::Buffer& tf_buffer) {
  std::vector<std::string> topics;
  topics.push_back("/tf_static");
  boost::shared_ptr<rosbag::View> view = mapping::queryBagfiles(bags, topics);
  for(rosbag::View::const_iterator iter = view->begin(); iter != view->end(); ++iter) {
    // Collect static tf messages so frame queries can be performed
    tf2_msgs::TFMessage::ConstPtr msg = iter->instantiate<tf2_msgs::TFMessage>();
    for(unsigned int i = 0; i < msg->transforms.size(); i++) {
      geometry_msgs::TransformStamped transform;
      tf2::convert(msg->transforms[i], transform);
      tf_buffer.setTransform(transform, "unknown", true);
    }
  }
}

/* ************************************************************************* */


/* ************************************************************************* */
TimeRange extractTimeRange(const std::vector<rosbag::Bag>& bags) {
  TimeRange time_range;

  // Check that at least one input bagfile was provided
  if(bags.empty()) {
    throw std::runtime_error("Must provide at least one source bag file.");
  }

  // Create query that combines all of the bagfiles together
  boost::shared_ptr<rosbag::View> view = queryBagfiles(bags);

  // Get the start and end times
  time_range.first  = view->getBeginTime();
  time_range.second = view->getEndTime();

  return time_range;
}

/* ************************************************************************* */
TfTransforms extractTfTransforms(const std::vector<rosbag::Bag>& bags, const TimeRange& time_range, const ros::Duration tf_buffer) {
  TfTransforms transforms;

  Timer timer;
  timer.start();

  // Check that at least one input bagfile was provided
  if(bags.empty()) {
    throw std::runtime_error("Must provide at least one source bag file.");
  }

  // Check that valid start and end times were provided
  if((time_range.first == ros::Time(0)) || (time_range.second == ros::Time(0)) || (time_range.first >= time_range.second)) {
    throw std::runtime_error("Must provide valid start and end times.");
  }

  // Define the start and end times with the tf buffer
  ros::Time tf_start_time = time_range.first - tf_buffer;
  ros::Time tf_end_time = time_range.second + tf_buffer;
  double tf_duration = (tf_end_time - tf_start_time).toSec();

  // Create a view of just the tf data
  std::vector<std::string> topics;
  topics.push_back("/tf");
  topics.push_back("/tf_static");
  boost::shared_ptr<rosbag::View> view = queryBagfiles(bags, topics, TimeRange(tf_start_time, tf_end_time));

  // Check for an empty view
  if(view->size() == 0) {
    throw std::runtime_error("No tf transforms present in the bag files.");
  }

  for(rosbag::View::const_iterator iter = view->begin(); iter != view->end(); ++iter) {

    // Skip timestamps outside of the specified range
    if(iter->getTime() < tf_start_time) continue;
    if(iter->getTime() > tf_end_time)   break;

    // Reconstitute the tf message
    tf2_msgs::TFMessage::ConstPtr msg = iter->instantiate<tf2_msgs::TFMessage>();

    // Collect tf messages so frame queries can be performed
    for(unsigned int i = 0; i < msg->transforms.size(); i++) {
      transforms.insert(TfTransforms::value_type(msg->transforms.at(i).header.stamp, msg->transforms.at(i)));
    }
  }

  // Add the results to the output
  timer.stop();
  ROS_DEBUG_STREAM("Extracted " << transforms.size() << " tf2_msgs::TFMessage between rostime " << tf_start_time << " and rostime " << tf_end_time << " in " << timer.elapsed() << " seconds.");

  return transforms;
}

/* ************************************************************************* */
Odometry extractOdometry(const std::vector<rosbag::Bag>& bags, const TimeRange& time_range, const std::string& topic) {
  Odometry odometry;

  Timer timer;
  timer.start();

  // Check that at least one input bagfile was provided
  if(bags.empty()) {
    throw std::runtime_error("Must provide at least one source bag file.");
  }

  // Check that valid start and end times were provided
  if((time_range.first == ros::Time(0)) || (time_range.second == ros::Time(0)) || (time_range.first >= time_range.second)) {
    throw std::runtime_error("Must provide valid start and end times.");
  }

  // Define the start and end times with a small buffer to account for the difference between the message timestamp and the header timestamp
  ros::Duration odom_buffer(1.0);
  ros::Time odom_start_time = time_range.first - odom_buffer;
  ros::Time odom_end_time = time_range.second + odom_buffer;
  double odom_duration = (odom_end_time - odom_start_time).toSec();

  // Create a view of just the odometry data
  std::vector<std::string> topics;
  topics.push_back(topic);
  boost::shared_ptr<rosbag::View> view = queryBagfiles(bags, topics, TimeRange(odom_start_time, odom_end_time));

  // Check for an empty view
  if(view->size() == 0) {
    throw std::runtime_error("No odometry present in the bag files.");
  }

  // Loop over the odometry messages, extracting all messages between the provided timestamps
  // Since odometry measurements will be calculated between consecuative timestamps, we also
  // need the message before the start time and the message after the end time.
  nav_msgs::Odometry::ConstPtr previous_odom_msg;
  nav_msgs::Odometry::ConstPtr current_odom_msg;
  size_t message_counter = 0;
  for(rosbag::View::const_iterator iter = view->begin(); iter != view->end(); ++iter) {

    // Update the previous message pointer
    previous_odom_msg = current_odom_msg;

    // Reconstitute the current odometry message
    current_odom_msg = iter->instantiate<nav_msgs::Odometry>();

    // Only add messages after the start time.
    if(current_odom_msg->header.stamp >= time_range.first) {
      // If this is the first entry after the start time, also add the previous message.
      if(odometry.empty() && previous_odom_msg) {
        odometry.insert(Odometry::value_type(previous_odom_msg->header.stamp, *previous_odom_msg));
      }
      // This message is after the start time. Add it to the container.
      odometry.insert(Odometry::value_type(current_odom_msg->header.stamp, *current_odom_msg));
    }
    // Stop once the end time is reached.
    if(current_odom_msg->header.stamp >= time_range.second) {
      // This message, which is after the end time, has already been added. Exit the loop.
      break;
    }
  }

  // Add the results to the output
  timer.stop();
  ROS_DEBUG_STREAM("Extracted " << odometry.size() << " nav_msgs::Odometry messages between rostime " << odom_start_time << " and rostime " << odom_end_time << " in " << timer.elapsed() << " seconds.");

  return odometry;
}

/* ************************************************************************* */
LaserScans extractLaserScans(const std::vector<rosbag::Bag>& bags, const TimeRange& time_range, const std::string& topic) {
  LaserScans scans;

  Timer timer;
  timer.start();

  // Check that at least one input bagfile was provided
  if(bags.empty()) {
    throw std::runtime_error("Must provide at least one source bag file.");
  }

  // Check that valid start and end times were provided
  if((time_range.first == ros::Time(0)) || (time_range.second == ros::Time(0)) || (time_range.first >= time_range.second)) {
    throw std::runtime_error("Must provide valid start and end times.");
  }

  // Define the start and end times with a small buffer to account for the difference between the message timestamp and the header timestamp
  ros::Duration scan_buffer(1.0);
  ros::Time scan_start_time = time_range.first - scan_buffer;
  ros::Time scan_end_time = time_range.second + scan_buffer;
  double scan_duration = (scan_end_time - scan_start_time).toSec();

  // Create a view of just the laserscan data
  std::vector<std::string> topics;
  topics.push_back(topic);
  boost::shared_ptr<rosbag::View> view = queryBagfiles(bags, topics, TimeRange(scan_start_time, scan_end_time));

  // Check for an empty view
  if(view->size() == 0) {
    throw std::runtime_error("No laserscans present in the bag files.");
  }

  for(rosbag::View::const_iterator iter = view->begin(); iter != view->end(); ++iter) {

    // Reconstitute the laser scan message
    sensor_msgs::LaserScan::ConstPtr scan = iter->instantiate<sensor_msgs::LaserScan>();

    // Skip timestamps outside of the specified range
    if(scan->header.stamp < time_range.first) continue;
    if(scan->header.stamp > time_range.second)   break;

    // Collect the laser scan messages
    scans.insert(LaserScans::value_type(scan->header.stamp, *scan));
  }

  // Add the results to the output
  timer.stop();
  ROS_DEBUG_STREAM("Extracted " << scans.size() << " sensor_msgs::LaserScan messages between rostime " << scan_start_time << " and rostime " << scan_end_time << " in " << timer.elapsed() << " seconds.");

  return scans;
}

/* ************************************************************************* */
PointClouds extractPointclouds(const std::vector<rosbag::Bag>& bags, const TimeRange& time_range, const std::string& topic) {
  PointClouds clouds;

  Timer timer;
  timer.start();

  // Check that at least one input bagfile was provided
  if(bags.empty()) {
    throw std::runtime_error("Must provide at least one source bag file.");
  }

  // Check that valid start and end times were provided
  if((time_range.first == ros::Time(0)) || (time_range.second == ros::Time(0)) || (time_range.first >= time_range.second)) {
    throw std::runtime_error("Must provide valid start and end times.");
  }

  // Define the start and end times with a small buffer to account for the difference between the message timestamp and the header timestamp
  ros::Duration cloud_buffer(1.0);
  ros::Time cloud_start_time = time_range.first - cloud_buffer;
  ros::Time cloud_end_time = time_range.second + cloud_buffer;
  double cloud_duration = (cloud_end_time - cloud_start_time).toSec();

  // Create a view of just the cloud data
  std::vector<std::string> topics;
  topics.push_back(topic);
  boost::shared_ptr<rosbag::View> view = queryBagfiles(bags, topics, TimeRange(cloud_start_time, cloud_end_time));

  // Check for an empty view
  if(view->size() == 0) {
    throw std::runtime_error("No pointclouds present in the bag files.");
  }

  for(rosbag::View::const_iterator iter = view->begin(); iter != view->end(); ++iter) {

    // Reconstitute the laser scan message
    sensor_msgs::PointCloud2::ConstPtr cloud = iter->instantiate<sensor_msgs::PointCloud2>();

    // Skip timestamps outside of the specified range
    if(cloud->header.stamp < time_range.first) continue;
    if(cloud->header.stamp > time_range.second)   break;

    // Collect the laser scan messages
    clouds.insert(PointClouds::value_type(cloud->header.stamp, *cloud));
  }

  // Add the results to the output
  timer.stop();
  ROS_DEBUG_STREAM("Extracted " << clouds.size() << " sensor_msgs::PointCloud2 messages between rostime " << cloud_start_time << " and rostime " << cloud_end_time << " in " << timer.elapsed() << " seconds.");

  return clouds;
}

/* ************************************************************************* */
PoseWithCovariances extractPoses(const std::vector<rosbag::Bag>& bags, const TimeRange& time_range, const std::string& topic, const boost::array<double, 36>& default_covariance) {
  PoseWithCovariances poses;

  Timer timer;
  timer.start();

  // Check that at least one input bagfile was provided
  if(bags.empty()) {
    throw std::runtime_error("Must provide at least one source bag file.");
  }

  // Check that valid start and end times were provided
  if((time_range.first == ros::Time(0)) || (time_range.second == ros::Time(0)) || (time_range.first >= time_range.second)) {
    throw std::runtime_error("Must provide valid start and end times.");
  }

  // Define the start and end times with a small buffer to account for the difference between the message timestamp and the header timestamp
  ros::Duration message_buffer(1.0);
  ros::Time message_start_time = time_range.first - message_buffer;
  ros::Time message_end_time = time_range.second + message_buffer;
  double message_duration = (message_end_time - message_start_time).toSec();

  // Create a view of just the pose data
  std::vector<std::string> topics;
  topics.push_back(topic);
  boost::shared_ptr<rosbag::View> view = queryBagfiles(bags, topics, TimeRange(message_start_time, message_end_time));

  // Check for an empty view
  if(view->size() == 0) {
    ROS_WARN_STREAM("No poses present in the bag files.");
  }

  // Loop over the pose messages, extracting all messages between the provided timestamps
  size_t message_counter = 0;
  for(rosbag::View::const_iterator iter = view->begin(); iter != view->end(); ++iter) {

    // Reconstitute the pose message (currently supported pose types are: PoseWithCovarianceStamped and PoseStamped)
    geometry_msgs::PoseStamped::ConstPtr pose_stamped = iter->instantiate<geometry_msgs::PoseStamped>();
    geometry_msgs::PoseWithCovarianceStamped::ConstPtr pose_with_covariance_stamped = iter->instantiate<geometry_msgs::PoseWithCovarianceStamped>();
    if(pose_stamped) {
      geometry_msgs::PoseWithCovarianceStamped pose_with_default_covariance_stamped;
      pose_with_default_covariance_stamped.header = pose_stamped->header;
      pose_with_default_covariance_stamped.pose.pose = pose_stamped->pose;
      pose_with_default_covariance_stamped.pose.covariance = default_covariance;
      pose_with_covariance_stamped = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>(pose_with_default_covariance_stamped);
    }
    // Skip timestamps outside of the specified range
    if(pose_with_covariance_stamped->header.stamp < time_range.first) continue;
    if(pose_with_covariance_stamped->header.stamp > time_range.second)   break;

    // Collect the pose messages
    poses.insert(PoseWithCovariances::value_type(pose_with_covariance_stamped->header.stamp, *pose_with_covariance_stamped));
  }

  // Add the results to the output
  timer.stop();
  ROS_DEBUG_STREAM("Extracted " << poses.size() << " geometry_msgs::PoseWithCovarianceStamped messages between rostime " << message_start_time << " and rostime " << message_end_time << " in " << timer.elapsed() << " seconds.");

  return poses;
}

/* ************************************************************************* */
Timestamps::const_iterator findClosest(const Timestamps& timestamps, const ros::Time& query_time, boost::optional<double> tolerance) {
  // Set the initial and final position of the search
  Timestamps::const_iterator timestamps_begin = timestamps.begin();
  Timestamps::const_iterator timestamps_end = timestamps.end();
  if(tolerance) {
    timestamps_begin = timestamps.lower_bound(query_time - ros::Duration(tolerance.get()));
    timestamps_end   = timestamps.upper_bound(query_time + ros::Duration(tolerance.get()));
  }

  // Perform the search
  double min_delta = std::numeric_limits<double>::max();
  Timestamps::const_iterator min_iter = timestamps.end();
  for(Timestamps::const_iterator iter = timestamps_begin; iter != timestamps_end; ++iter) {
    // Compute the time delta to the query timestamp
    double delta = std::fabs( (*iter - query_time).toSec() );
    // If the time delta is smaller, keep this one
    if(delta < min_delta) {
      min_delta = delta;
      min_iter = iter;
    }
    // If the time delta is larger, we are through searching (the map is sorted)
    if(delta > min_delta) {
      break;
    }
  }

  // Return the iterator with the closest timestamp
  // If the container is empty, or no entries exist within the provided time tolerance, this iterator will point to timestamps.end()
  return min_iter;
}

/* ************************************************************************* */
Poses computePoses(const std::set<ros::Time>& timestamps, const TfTransforms& transforms, const std::string& global_frame, const std::string& base_frame) {
  Poses poses;
  static const ros::Duration tf_buffer(5.0);

  Timer timer;
  timer.start();

  // Loop over the scans
  size_t counter = 0;
  tf2::BufferCore tf_transformer(tf_buffer+tf_buffer);
  TfTransforms::const_iterator tf_begin = transforms.begin();
  TfTransforms::const_iterator tf_end = transforms.begin();
  for(std::set<ros::Time>::const_iterator iter = timestamps.begin(); iter != timestamps.end(); ++iter) {

    // Quick access to the timestamp
    const ros::Time& timestamp = *iter;

    // Query the transforms around the scan time
    tf_begin = tf_end;
    tf_end = transforms.upper_bound(timestamp + tf_buffer);

    // Add the new transforms to the tf listener
    for(TfTransforms::const_iterator tf = tf_begin; tf != tf_end; ++tf) {
      tf_transformer.setTransform(tf->second, "default_authority");
    }

    // Check if the odom pose is available
    bool pose_available = tf_transformer.canTransform(global_frame, base_frame, timestamp);
    if(!pose_available) {
      //throw(std::runtime_error("Pose not available for time: " + boost::lexical_cast<std::string>(timestamp.toSec())));
      ROS_WARN_STREAM("Pose not available for time: " << timestamp << " (Attempting to find transform from '" << base_frame << "' to '" << global_frame << "' at timestamp " << timestamp << ").");
      continue;
    }

    // Get the pose from the tf listener
    geometry_msgs::TransformStamped stamped_pose = tf_transformer.lookupTransform(global_frame, base_frame, timestamp);

    // Extract the 2D pose form the transform
    gtsam::Pose2 pose(stamped_pose.transform.translation.x, stamped_pose.transform.translation.y, tf::getYaw(stamped_pose.transform.rotation));
    poses.insert(Poses::value_type(timestamp, pose));
  }

  // Add the results to the output
  timer.stop();
  ROS_DEBUG_STREAM("Computed " << poses.size() << " gtsam::Pose2 objects at " << timestamps.size() << " timestamps in " << timer.elapsed() << " seconds.");

  return poses;
}

///* ************************************************************************* */
//void writePoses(const std::string& filename, const Poses& poses) {
//
//  // create an output file
//  boost::filesystem::path file_path(filename);
//  boost::filesystem::create_directories(file_path.parent_path());
//  std::ofstream logfile(filename.c_str());
//
//  // Loop through each variable, writing the output to disk
//  BOOST_FOREACH(const Poses::value_type& timestamp_pose, poses) {
//    logfile << timestamp_pose.first << " " << timestamp_pose.second.x() << " " << timestamp_pose.second.y() << " " << timestamp_pose.second.theta();
//    logfile << std::endl;
//  }
//
//  // Close the logfile
//  logfile.close();
//}

/* ************************************************************************* */
void writePointcloudPLY(const PointClouds& clouds, const std::string& filename, bool use_binary_format) {

  // Check for at least one pointcloud to write
  if(clouds.empty()) {
    throw(std::runtime_error("No pointclouds available to write to output PLY file"));
  }

  // create an output file
  boost::filesystem::create_directories(boost::filesystem::path(filename).parent_path());
  std::ofstream logfile(filename.c_str(), std::ios_base::binary);
  if(!logfile.is_open()) {
    throw(std::runtime_error("Could not open output file '" + filename + "'"));
  }

  // Compute the total number of points
  size_t point_count = 0;
  for(PointClouds::const_iterator iter = clouds.begin(); iter != clouds.end(); ++iter) {
    const sensor_msgs::PointCloud2& cloud = iter->second;
    point_count += cloud.height*cloud.width;
  }

  // Write header
  logfile << "ply" << std::endl;
  if(use_binary_format) {
    logfile << "format binary_little_endian 1.0" << std::endl;
  } else {
    logfile << "format ascii 1.0" << std::endl;
  }
  logfile << "element vertex " << point_count << std::endl;
  logfile << "property float x" << std::endl;
  logfile << "property float y" << std::endl;
  logfile << "property float z" << std::endl;
  logfile << "property float scan_id" << std::endl;
  logfile << "property double time" << std::endl;
  logfile << "end_header" << std::endl;

  // Loop through each scan, writing the output to disk
  for(PointClouds::const_iterator iter = clouds.begin(); iter != clouds.end(); ++iter) {

    const ros::Time& cloud_time = iter->first;
    const sensor_msgs::PointCloud2& cloud = iter->second;

    for(size_t m = 0; m < cloud.height; ++m) {
      for(size_t n = 0; n < cloud.width; ++n) {
        float x = *((float*)(&cloud.data[m*cloud.row_step + n*cloud.point_step + 0*sizeof(float)]));
        float y = *((float*)(&cloud.data[m*cloud.row_step + n*cloud.point_step + 1*sizeof(float)]));
        float z = *((float*)(&cloud.data[m*cloud.row_step + n*cloud.point_step + 2*sizeof(float)]));
        float sequence = cloud.header.seq;
        double timestamp = cloud.header.stamp.toSec();
        if(use_binary_format) {
          logfile.write((char*)&x, sizeof(x));
          logfile.write((char*)&y, sizeof(y));
          logfile.write((char*)&z, sizeof(z));
          logfile.write((char*)&sequence, sizeof(sequence));
          logfile.write((char*)&timestamp, sizeof(timestamp));
        } else {
          logfile    << std::setprecision(20) << x
              << " " << std::setprecision(20) << y
              << " " << std::setprecision(20) << z
              << " " << std::setprecision(20) << sequence
              << " " << std::setprecision(20) << timestamp
              << std::endl;
        }
      }
    }
  }

  // Close the logfile
  logfile.close();
}

/* ************************************************************************* */
void writePointcloudCSV(const PointClouds& clouds, const std::string& filename) {

  // Check for at least one pointcloud to write
  if(clouds.empty()) {
    throw(std::runtime_error("No pointclouds available to write to output CSV file"));
  }

  // create an output file
  boost::filesystem::create_directories(boost::filesystem::path(filename).parent_path());
  std::ofstream logfile(filename.c_str());
  if(!logfile.is_open()) {
    throw(std::runtime_error("Could not open output file '" + filename + "'"));
  }

  // Write header
  logfile << "%x,y,z,scan_id,time" << std::endl;

  // Loop through each scan, writing the output to disk
  for(PointClouds::const_iterator iter = clouds.begin(); iter != clouds.end(); ++iter) {

    const ros::Time& cloud_time = iter->first;
    const sensor_msgs::PointCloud2& cloud = iter->second;

    for(size_t m = 0; m < cloud.height; ++m) {
      for(size_t n = 0; n < cloud.width; ++n) {
        float x = *((float*)(&cloud.data[m*cloud.row_step + n*cloud.point_step + 0*sizeof(float)]));
        float y = *((float*)(&cloud.data[m*cloud.row_step + n*cloud.point_step + 1*sizeof(float)]));
        float z = *((float*)(&cloud.data[m*cloud.row_step + n*cloud.point_step + 2*sizeof(float)]));
        float sequence = cloud.header.seq;
        double timestamp = cloud.header.stamp.toSec();
        logfile    << std::setprecision(20) << x
            << "," << std::setprecision(20) << y
            << "," << std::setprecision(20) << z
            << "," << std::setprecision(20) << sequence
            << "," << std::setprecision(20) << timestamp
            << std::endl;
      }
    }
  }

  // Close the logfile
  logfile.close();
}

/* ************************************************************************* */
} /// @namespace bnr_mapping


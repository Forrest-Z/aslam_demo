/**
 * laserscan_processing.cpp
 */

#include <aslam_demo/mapping/laserscan_processing.h>
#include <aslam_demo/mapping/timer.h>
#include <aslam_demo/factors/laser_scan_factor.h>
#include <aslam_demo/factors/key_generator.h>
#include <laser_geometry/laser_geometry.h>
#include <filters/filter_chain.h>
//#include <yaml-cpp/yaml.h>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/transform.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometry.hpp>

namespace mapping {

namespace laserscan {

typedef boost::geometry::model::d2::point_xy<double, boost::geometry::cs::cartesian> PointType;
typedef boost::geometry::model::polygon<PointType> PolygonType;

/* ************************************************************************* */
/*LaserScans sparsifyLaserScans(const LaserScans& scans, const Poses& poses, double time_delta_threshold, double distance_delta_threshold, double rotation_delta_threshold) {
  LaserScans sparse_scans;

  Timer timer;
  timer.start();

  bool extracted_first_scan = false;
  ros::Time previous_time = scans.begin()->first;
  gtsam::Pose2 previous_pose = poses.at(previous_time);
  const sensor_msgs::LaserScan& previous_scan = scans.at(previous_time);
  size_t scan_counter = 0;
  for(LaserScans::const_iterator iter = scans.begin(); iter != scans.end(); ++iter) {

    // Quick access to the scan
    ros::Time current_time = iter->first;
    const sensor_msgs::LaserScan& current_scan = iter->second;

    // Look up the pose
    Poses::const_iterator pose_iter = poses.find(current_time);
    if(pose_iter == poses.end()) {
      //throw std::runtime_error("Could not find a matching pose for this laserscan.");
      ROS_WARN_STREAM("Could not find a matching pose for this laserscan. Skipping.");
      continue;
    }
    const gtsam::Pose2& current_pose = pose_iter->second;

    // Compute the time, distance, rotation change since the last extracted pose
    double time_delta = (current_time - previous_time).toSec();
    double distance_delta = current_pose.translation().distance(previous_pose.translation());
    double rotation_delta = current_pose.rotation().between(previous_pose.rotation()).theta();

    // If any of the computed deltas exceed the configured thresholds, keep the scan
    if(    !extracted_first_scan
        || (time_delta >= time_delta_threshold)
        || (distance_delta >= distance_delta_threshold)
        || (rotation_delta >= rotation_delta_threshold) ) {
      // Keep this scan
      sparse_scans.insert(LaserScans::value_type(current_time, current_scan));
      extracted_first_scan = true;
      previous_time = current_time;
      previous_pose = current_pose;
    }
  }

  // Add the results to the output
  timer.stop();
  ROS_DEBUG_STREAM("Sparsified " << scans.size() << " sensor_msgs::LaserScan messages down to " << sparse_scans.size() << " messages in " << timer.elapsed() << " seconds.");

  return sparse_scans;
}
*/
/* ************************************************************************* */
/*XmlRpc::XmlRpcValue readFilterConfiguration(const std::string& filename) {
  XmlRpc::XmlRpcValue xml_filters;

  // Error checking
  if(!boost::filesystem::exists(filename)) {
    throw std::runtime_error("LaserScan Filter configuration file does not exist.");
  }

  // Parse the document assuming everything is correct. Catch any exceptions throw
  try {
    YAML::Node yaml_filters = YAML::LoadFile(filename);

    size_t filter_count = 0;
    for(YAML::const_iterator yaml_filter_iter = yaml_filters.begin(); yaml_filter_iter != yaml_filters.end(); ++yaml_filter_iter) {
      const YAML::Node& yaml_filter = *yaml_filter_iter;
      XmlRpc::XmlRpcValue xml_filter;

      // Extract the filter name
      std::string filter_name;
      filter_name = yaml_filter["name"].as<std::string>();
      xml_filter["name"] = filter_name;

      // Extract the filter type
      std::string filter_type;
      filter_type = yaml_filter["type"].as<std::string>();
      xml_filter["type"] = filter_type;

      // Extract the filter params
      if(yaml_filter["params"]) {
        const YAML::Node& yaml_params = yaml_filter["params"];
        XmlRpc::XmlRpcValue xml_params;

        for(YAML::const_iterator yaml_param_iter = yaml_params.begin(); yaml_param_iter != yaml_params.end(); ++yaml_param_iter) {
          std::string key = yaml_param_iter->first.as<std::string>();
          try {
            int value = yaml_param_iter->second.as<int>();
            xml_params[key] = value;
          } catch(const std::exception& e) {
            double value = yaml_param_iter->second.as<double>();
            xml_params[key] = value;
          }
        }
        xml_filter["params"] = xml_params;
      }

      // Add filter to the filters collection
      xml_filters[filter_count++] = xml_filter;
    }
  } catch(const std::exception& e) {
    throw std::runtime_error("Failed to parse scan filter YAML file. Error: " + std::string(e.what()));
  }

  return xml_filters;
}
*/
/* ************************************************************************* */
/*LaserScans filterLaserScans(const LaserScans& scans, const std::string& filename) {
  LaserScans filtered_scans;

  Timer timer;
  timer.start();

  if(scans.empty()) {
    throw std::runtime_error("No scans available to filter.");
  }

  // Read the YAML configuration file
  XmlRpc::XmlRpcValue laser_filter_config = readFilterConfiguration(filename);

  // Create Laser Scan Filters
  filters::FilterChain<sensor_msgs::LaserScan> filter_chain("sensor_msgs::LaserScan");
  bool filters_valid = filter_chain.configure(laser_filter_config, "root");
  if(!filters_valid) {
    throw std::runtime_error("Could not configure the scan filter chain.");
  }

  size_t scan_counter = 0;
  for(LaserScans::const_iterator iter = scans.begin(); iter != scans.end(); ++iter) {

    // Quick access to the scan
    const ros::Time& scan_time = iter->first;
    const sensor_msgs::LaserScan& scan = iter->second;

    // Apply the filter chain
    sensor_msgs::LaserScan filtered_scan;
    filter_chain.update(scan, filtered_scan);
    filtered_scans.insert(LaserScans::value_type(filtered_scan.header.stamp, filtered_scan));
  }

  // Add the results to the output
  timer.stop();
  ROS_DEBUG_STREAM("Filtered " << filtered_scans.size() << " sensor_msgs::LaserScan messages in " << timer.elapsed() << " seconds.");

  return filtered_scans;
}

/* ************************************************************************* */
/*PointClouds computePointClouds(const LaserScans& scans, const TfTransforms& transforms, const std::string& target_frame) {
  PointClouds clouds;
  static const ros::Duration tf_buffer(5.0);

  Timer timer;
  timer.start();

  if(scans.empty()) {
    throw std::runtime_error("No scans available to convert to pointclouds.");
  }
  if(transforms.empty()) {
    throw std::runtime_error("No transforms available for pointcloud conversion.");
  }

  // Loop over the scans
  tf2::BufferCore tf_transformer(tf_buffer+tf_buffer);
  laser_geometry::LaserProjection laser_projector;
  TfTransforms::const_iterator tf_begin = transforms.begin();
  TfTransforms::const_iterator tf_end = transforms.begin();
  for(LaserScans::const_iterator iter = scans.begin(); iter != scans.end(); ++iter) {

    // Quick access to the scan
    const ros::Time& scan_time = iter->first;
    const sensor_msgs::LaserScan& scan = iter->second;

    // Query the transforms around the scan time
    tf_begin = tf_end;
    tf_end = transforms.upper_bound(scan_time + tf_buffer);

    // Add the new transformes to the tf listener
    for(TfTransforms::const_iterator tf = tf_begin; tf != tf_end; ++tf) {
      tf_transformer.setTransform(tf->second, "default_authority");
    }

    // Check if the transformation is possible
    bool transform_available = tf_transformer.canTransform(target_frame, scan.header.frame_id, scan.header.stamp)
                       && tf_transformer.canTransform(target_frame, scan.header.frame_id, scan.header.stamp + ros::Duration(scan.scan_time));

    if(!transform_available) {
      ROS_ERROR_STREAM("Cannot compute pointcloud from laserscan at time: " << scan.header.stamp << " (Attempting to find transform from '" << scan.header.frame_id << "' to '" << target_frame << "' at timestamp " << scan.header.stamp << ").");
      //throw(std::runtime_error("Cannot compute pointcloud from laserscan at time: " + boost::lexical_cast<std::string>(scan->header.stamp.toSec())));
    }

    if(transform_available) {
      // Convert laser scan into a pointcloud using the tf system
      sensor_msgs::PointCloud2 cloud;
      laser_projector.transformLaserScanToPointCloud(target_frame, scan, cloud, tf_transformer);
      clouds.insert(PointClouds::value_type(cloud.header.stamp, cloud));
    }
  }

  timer.stop();
  ROS_DEBUG_STREAM("Computed " << clouds.size() << " sensor_msgs::PointCloud2 messages in " << timer.elapsed() << " seconds.");

  return clouds;
}
*/
/* ************************************************************************* */
/*AugmentedLaserScans createAugmentedLaserScans(const Poses& odom_poses, const Poses& map_poses, const LaserScans& scans, const PointClouds& clouds) {
  AugmentedLaserScans augmented_scans;

  Timer timer;
  timer.start();

  if(scans.empty()) {
    //throw std::runtime_error("No scans available to convert to augmented scans.");
    ROS_WARN_STREAM("No scans available to convert to augmented scans.");
  }

  // Loop over the input laserscans, creating an augmented scan
  for(LaserScans::const_iterator iter = scans.begin(); iter != scans.end(); ++iter) {

    // Quick access to the scan
    const ros::Time& scan_time = iter->first;
    const sensor_msgs::LaserScan& scan = iter->second;

    // Look up the odom pose
    Poses::const_iterator odom_pose_iter = odom_poses.find(scan_time);
    if(odom_pose_iter == odom_poses.end()) {
      //throw std::runtime_error("Could not find a matching odometry pose to use in the augmented laserscan.");
      ROS_WARN_STREAM("Could not find a matching odometry pose to use in the augmented laserscan at timestamp " << scan_time << ".");
      continue;
    }
    const gtsam::Pose2& odom_pose = odom_pose_iter->second;

    // Look up the global pose
    Poses::const_iterator map_pose_iter = map_poses.find(scan_time);
    if(map_pose_iter == map_poses.end()) {
      //throw std::runtime_error("Could not find a matching map pose to use in the augmented laserscan.");
      ROS_WARN_STREAM("Could not find a matching map pose to use in the augmented laserscan at timestamp " << scan_time << ".");
      continue;
    }
    const gtsam::Pose2& map_pose = map_pose_iter->second;

    // Look up the pointcloud
    PointClouds::const_iterator clouds_iter = clouds.find(scan_time);
    if(clouds_iter == clouds.end()) {
      //throw std::runtime_error("Could not find a matching pointcloud to use in the augmented laserscan.");
      ROS_WARN_STREAM("Could not find a matching pointcloud to use in the augmented laserscan at timestamp " << scan_time << ".");
      continue;
    }
    const sensor_msgs::PointCloud2& cloud = clouds_iter->second;

    AugmentedLaserScan augmented_scan;
    augmented_scan.odom_pose = odom_pose;
    augmented_scan.map_pose = map_pose;
    augmented_scan.scan = scan;
    augmented_scan.cloud = cloud;
    augmented_scans[scan_time] = augmented_scan;
  }

  timer.stop();
  ROS_DEBUG_STREAM("Computed " << augmented_scans.size() << " augmented laserscan structures in " << timer.elapsed() << " seconds.");

  return augmented_scans;
}
*/
///* ************************************************************************* */
//double ComputeOverlap(const gtsam::Pose2& pose1, const gtsam::Pose2& pose2, double angle_min, double angle_max, double range_max) {
//  // Construct a laserscan polygon
//  // Note that the points should be added counter-clockwise
//  size_t vertex_count = 10;
//  double angle_delta = (angle_max - angle_min) / (vertex_count - 1);
//  PolygonType laserscan_polygon;
//  boost::geometry::append(laserscan_polygon, PointType(0.0, 0.0));
//  for(size_t i = 0; i < vertex_count; ++i) {
//    double angle = angle_max - i*angle_delta;
//    PointType point(range_max*cos(angle), range_max*sin(angle));
//    boost::geometry::append(laserscan_polygon, point);
//  }
//  boost::geometry::append(laserscan_polygon, PointType(0.0, 0.0));
//  double total_area = boost::geometry::area(laserscan_polygon);
//
//  // Transform the laserscan polygon to the first robot pose
//  PolygonType laserscan_polygon1;
//  {
//    PolygonType tmp;
//    boost::geometry::strategy::transform::rotate_transformer  <PointType, PointType, boost::geometry::radian> rotate(pose1.theta());
//    boost::geometry::transform(laserscan_polygon, tmp, rotate);
//
//    boost::geometry::strategy::transform::translate_transformer<PointType, PointType> translate(pose1.x(), pose1.y());
//    boost::geometry::transform(tmp, laserscan_polygon1, translate);
//  }
//
//  // Transform the laserscan polygon to the second robot pose
//  PolygonType laserscan_polygon2;
//  {
//    PolygonType tmp;
//    boost::geometry::strategy::transform::rotate_transformer  <PointType, PointType, boost::geometry::radian> rotate(pose2.theta());
//    boost::geometry::transform(laserscan_polygon, tmp, rotate);
//
//    boost::geometry::strategy::transform::translate_transformer<PointType, PointType> translate(pose2.x(), pose2.y());
//    boost::geometry::transform(tmp, laserscan_polygon2, translate);
//  }
//
//  // Compute the intersection polygon area
//  std::vector<PolygonType> overlap_polygons;
//  boost::geometry::intersection(laserscan_polygon1, laserscan_polygon2, overlap_polygons);
//  double overlap_area = 0;
//  for(size_t polygon_index = 0;  polygon_index < overlap_polygons.size(); ++polygon_index) {
//    overlap_area += boost::geometry::area(overlap_polygons.at(polygon_index));
//  }
//
//  return overlap_area / total_area;
//}

///* ************************************************************************* */
//double ComputeOverlap(const gtsam::Pose2& pose1, const gtsam::Pose2& pose2, const sensor_msgs::LaserScan& scan1, const sensor_msgs::LaserScan& scan2) {
//
//  // Create the laserscan polygon for the first robot pose
//  PolygonType laserscan_polygon1;
//  double laserscan_polygon1_area = 0;
//  {
//    // Create the actual laserscan
//    boost::geometry::append(laserscan_polygon1, PointType(pose1.x(), pose1.y()));
//    for(int i = scan1.ranges.size()-1; i >= 0; --i) {
//      if((std::isnan(scan1.ranges.at(i))) || (scan1.ranges.at(i) < scan1.range_min) || (scan1.ranges.at(i) > scan1.range_max)) {
//        continue;
//      }
//      double angle = scan1.angle_min + i*scan1.angle_increment + pose1.theta();
//      PointType point(scan1.ranges.at(i)*cos(angle) + pose1.x(), scan1.ranges.at(i)*sin(angle) + pose1.y());
//      boost::geometry::append(laserscan_polygon1, point);
//    }
//    boost::geometry::append(laserscan_polygon1, PointType(pose1.x(), pose1.y()));
//    laserscan_polygon1_area = boost::geometry::area(laserscan_polygon1);
//  }
//
//  // Create the laserscan polygon for the second robot pose
//  PolygonType laserscan_polygon2;
//  double laserscan_polygon2_area = 0;
//  {
//    // Create the actual laserscan
//    boost::geometry::append(laserscan_polygon2, PointType(pose2.x(), pose2.y()));
//    for(int i = scan2.ranges.size()-1; i >= 0; --i) {
//      if((std::isnan(scan2.ranges.at(i))) || (scan2.ranges.at(i) < scan2.range_min) || (scan2.ranges.at(i) > scan2.range_max)) {
//        continue;
//      }
//      double angle = scan2.angle_min + i*scan2.angle_increment + pose2.theta();
//      PointType point(scan2.ranges.at(i)*cos(angle) + pose2.x(), scan2.ranges.at(i)*sin(angle) + pose2.y());
//      boost::geometry::append(laserscan_polygon2, point);
//    }
//    boost::geometry::append(laserscan_polygon2, PointType(pose2.x(), pose2.y()));
//    laserscan_polygon2_area = boost::geometry::area(laserscan_polygon2);
//  }
//
//  // Compute the intersection polygon area
//  std::vector<PolygonType> overlap_polygons;
//  boost::geometry::intersection(laserscan_polygon1, laserscan_polygon2, overlap_polygons);
//  double overlap_area = 0;
//  for(size_t polygon_index = 0; polygon_index < overlap_polygons.size(); ++polygon_index) {
//    overlap_area += boost::geometry::area(overlap_polygons.at(polygon_index));
//  }
//
//  return overlap_area / std::min(laserscan_polygon1_area, laserscan_polygon2_area);
//}

/* ************************************************************************* */
/*TimestampPairs computeAugmentedLaserScanPairs(const AugmentedLaserScans& augmented_scans, double time_delta_threshold, double distance_delta_threshold, double rotation_delta_threshold) {
  TimestampPairs pairs;

  Timer timer;
  timer.start();

  if(augmented_scans.empty()) {
    throw std::runtime_error("No augmented scans available to compute laserscan pairs.");
  }

  // Set the first keyframe to be the first scan
  ros::Time keyframe_timestamp = augmented_scans.begin()->first;
  gtsam::Pose2 keyframe_pose = augmented_scans.begin()->second.odom_pose;

  // Start iterating from the second entry (since the first relative pose is between the first and second entry)
  AugmentedLaserScans::const_iterator iter = augmented_scans.begin();
  ++iter;

  size_t counter = 0;
  for(; iter != augmented_scans.end(); ++iter) {
    // Easy access for the current iterator data
    const ros::Time& current_timestamp = iter->first;
    const gtsam::Pose2& current_pose = iter->second.odom_pose;

    // Store the current pair
    pairs.push_back(TimestampPair(keyframe_timestamp, current_timestamp));

    // Determine when to select a new keyframe
    double time_delta = (current_timestamp - keyframe_timestamp).toSec();
    gtsam::Pose2 pose_delta = keyframe_pose.between(current_pose);
    double distance_delta = pose_delta.translation().norm();
    double rotation_delta = std::fabs(pose_delta.rotation().theta());
    if(  (time_delta >= time_delta_threshold)
      || (distance_delta >= distance_delta_threshold)
      || (rotation_delta >= rotation_delta_threshold) ) {
      keyframe_timestamp = current_timestamp;
      keyframe_pose = current_pose;
    }
  }

  // Add the results to the output
  timer.stop();
  ROS_DEBUG_STREAM("Computed " << pairs.size() << " augmented laserscan pairs in " << timer.elapsed() << " seconds.");

  return pairs;
}
*/
/* ************************************************************************* */
/*TimestampPairs computeAugmentedLaserScanPairs(const AugmentedLaserScans& augmented_scans,
    double minimum_time_delta, double maximum_time_delta,
    double minimum_x_delta, double maximum_x_delta,
    double minimum_y_delta, double maximum_y_delta,
    double minimum_rotation_delta, double maximum_rotation_delta) {

  TimestampPairs pairs;

  Timer timer;
  timer.start();

  if(augmented_scans.empty()) {
    throw std::runtime_error("No augmented scans available to compute laserscan pairs.");
  }

  // Loop over the augmented scans, looking for map poses that are far apart in time, but close together in space
  size_t counter = 0;
  for(AugmentedLaserScans::const_iterator scan1_iter = augmented_scans.begin(); scan1_iter != augmented_scans.end(); ++scan1_iter) {
    // Quick reference to the pose1 information
    const ros::Time& timestamp1 = scan1_iter->first;
    const gtsam::Pose2& map_pose1 = scan1_iter->second.map_pose;

    // Look for loop closure poses
    for(AugmentedLaserScans::const_iterator scan2_iter = scan1_iter; scan2_iter != augmented_scans.end(); ++scan2_iter) {
      // Quick reference to the pose2 information
      const ros::Time& timestamp2 = scan2_iter->first;
      const gtsam::Pose2& map_pose2 = scan2_iter->second.map_pose;

      // Determine when to keep a pair
      gtsam::Pose2 pose_delta = map_pose1.between(map_pose2);
      double time_delta = (timestamp2 - timestamp1).toSec();
      double x_delta = pose_delta.x();
      double y_delta = pose_delta.y();
      double rotation_delta = pose_delta.theta();
      if(  (time_delta >= minimum_time_delta) && (time_delta <= maximum_time_delta)
        && (x_delta >= minimum_x_delta) && (x_delta <= maximum_x_delta)
        && (y_delta >= minimum_y_delta) && (y_delta <= maximum_y_delta)
        && (    ((minimum_rotation_delta <  maximum_rotation_delta) && (rotation_delta >= minimum_rotation_delta) && (rotation_delta <= maximum_rotation_delta))
             || ((minimum_rotation_delta >= maximum_rotation_delta) && ((rotation_delta >= minimum_rotation_delta) || (rotation_delta <= maximum_rotation_delta))) ) ) {
        // Store the current pair
        pairs.push_back(TimestampPair(timestamp1, timestamp2));
      }
    }
  }

  // Add the results to the output
  timer.stop();

  return pairs;
}
*/
/* ************************************************************************* */
gtsam::NonlinearFactorGraph createLaserScanFactors(const RelativePoseEstimates& matches, double time_tolerance) {
  gtsam::NonlinearFactorGraph factors;

  Timer timer;
  timer.start();

  if(matches.empty()) {
    throw std::runtime_error("No scan matches available for factor creation.");
  }

  // Create a GTSAM Key Generator for creating variable names
  factors::KeyGenerator key_generator(time_tolerance);

  // Add the initial pose to the values structure
  gtsam::Key key = key_generator.generateKey(factors::key_type::Pose2, matches.front().timestamp1);

  // Create a factor for each match
  for(size_t i = 0; i < matches.size(); ++i) {

    // Quick access
    const RelativePoseEstimate& match = matches[i];

    // Check for a valid match
    if(match.timestamp1 == ros::Time() || match.timestamp2 == ros::Time()) {
      continue;
    }

    // Create a factor based on this relative pose and cov
    gtsam::Key key1 = key_generator.generateKey(factors::key_type::Pose2, match.timestamp1);
    gtsam::Key key2 = key_generator.generateKey(factors::key_type::Pose2, match.timestamp2);
    const gtsam::Pose2 relative_pose = match.relative_pose;
    gtsam::noiseModel::Base::shared_ptr noise_model(gtsam::noiseModel::Gaussian::Covariance(match.cov, true));
    gtsam::NonlinearFactor::shared_ptr factor(new factors::LaserScanFactor(key1, key2, relative_pose, noise_model));


    // Add the factor to the container. (Empty factors are added to keep the various containers synchronized)
    factors.push_back(factor);
  }

  // Add the results to the output
  timer.stop();
  ROS_DEBUG_STREAM("Computed " << factors.size() << " laserscan factors in " << timer.elapsed() << " seconds.");

  return factors;
}

///* ************************************************************************* */
//gtsam::Values CreateLaserscanValues(const RelativePoseEstimates& matches, const gtsam::Pose2& initial_pose, double time_tolerance) {
//  gtsam::Values values;
//
//  Timer timer;
//  timer.start();
//
//  if(matches.empty()) {
//    throw std::runtime_error("No scan matches available for initial values creation.");
//  }
//
//  // Create a GTSAM Key Generator for creating variable names
//  factors::KeyGenerator key_generator(time_tolerance);
//
//  // Add the initial pose to the values structure
//  gtsam::Key key = key_generator.generateKey(factors::key_type::Pose2, matches.front().timestamp1);
//  values.insert(key, initial_pose);
//
//  // Create a factor for each match
//  for(size_t i = 0; i < matches.size(); ++i) {
//
//    // Quick access
//    const RelativePoseEstimate& match = matches[i];
//
//    // Check if this is a valid match
//    if(match.timestamp1 != ros::Time() && match.timestamp2 != ros::Time()) {
//      // Create a factor from the relative pose and cov
//      gtsam::Key key1 = key_generator.generateKey(factors::key_type::Pose2, match.timestamp1);
//      gtsam::Key key2 = key_generator.generateKey(factors::key_type::Pose2, match.timestamp2);
//      const gtsam::Pose2 relative_pose = match.relative_pose;
//
//      // Add the initial guess for the latest pose
//      gtsam::Pose2 pose1 = values.at<gtsam::Pose2>(key1);
//      gtsam::Pose2 pose2 = pose1.compose(relative_pose);
//      values.insert(key2, pose2);
//    }
//
//    // Print a progress message
//    printProgressBar("  Creating Initial Values: ", 100 * (double)(i) / matches.size());
//  }
//
//  // Add the results to the output
//  timer.stop();
//  std::cout << ", Values: " << values.size() << ", Time: " << timer.elapsed() << std::endl;
//
//  return values;
//}

///* ************************************************************************* */
//void WriteLaserscanPoints(const Scans& scans, const Clouds& clouds, const Poses& poses, const std::string& filename) {
//
//  Timer timer;
//  timer.start();
//
//  // create an output file
//  boost::filesystem::path file_path(filename);
//  boost::filesystem::create_directories(file_path.parent_path());
//  std::ofstream logfile(filename.c_str());
//
//  // Loop through each scan, writing the output to disk
//  size_t scan_counter = 0;
//  for(Scans::const_iterator iter = scans.begin(); iter != scans.end(); ++iter) {
//    // Get the current timestamp
//    ros::Time scan_time = iter->first;
//
//    // Look up the scan, pose, and pointcloud for that time
//    Scans::const_iterator scan_iter = scans.find(scan_time);
//    Clouds::const_iterator cloud_iter = clouds.find(scan_time);
//    Poses::const_iterator pose_iter = poses.find(scan_time);
//
//    // Check that all required information was found
//    if(    (scan_iter != scans.end())
//        && (cloud_iter != clouds.end())
//        && (pose_iter != poses.end())
//    ) {
//      // Quick access to the data members
//      const sensor_msgs::PointCloud2& cloud = cloud_iter->second;
//      gtsam::Pose2 pose = pose_iter->second;
//
//      logfile << scan_time;
//      logfile << " " << pose.x() << " " << pose.y() << " " << pose.theta();
//      for(size_t m = 0; m < cloud.height; ++m) {
//        for(size_t n = 0; n < cloud.width; ++n) {
//          float x = *((float*)(&cloud.data[m*cloud.row_step + n*cloud.point_step + 0*sizeof(float)]));
//          float y = *((float*)(&cloud.data[m*cloud.row_step + n*cloud.point_step + 1*sizeof(float)]));
//          logfile << " " << x << " " << y;
//        }
//      }
//      logfile << std::endl;
//    }
//
//    // Print a progress message
//    printProgressBar("  Writing LaserScan Points Log: ", 100 * (double)(scan_counter++) / scans.size());
//  }
//
//  // Close the logfile
//  logfile.close();
//
//  // Add the results to the output
//  timer.stop();
//  std::cout << ", LaserScans: " << scans.size() << ", Time: " << timer.elapsed() << std::endl;
//}
//
///* ************************************************************************* */
//void WriteLaserscanPairs(const TimestampPairs& pairs, const AugmentedScans& augmented_scans, const std::string& debug_log_path) {
//
//  Timer timer;
//  timer.start();
//
//  // create an output directory
//  boost::filesystem::path file_path(debug_log_path);
//  boost::filesystem::create_directories(file_path);
//
//  // Print a progress message
//  printProgressBar("  Writing LaserScan Pairs Log: ", 0);
//
//  // Loop through each scan, writing the output to disk
//  size_t pair_counter = 0;
//  for(TimestampPairs::const_iterator iter = pairs.begin(); iter != pairs.end(); ++iter) {
//    // Get the current timestamp
//    ros::Time timestamp1 = iter->first;
//    ros::Time timestamp2 = iter->second;
//
//    // Look up the augmented scan for each timestamp
//    AugmentedScans::const_iterator scan1_iter = augmented_scans.find(timestamp1);
//    AugmentedScans::const_iterator scan2_iter = augmented_scans.find(timestamp2);
//
//    // Check that all required information was found
//    if(    (scan1_iter != augmented_scans.end())
//        && (scan2_iter != augmented_scans.end())
//    ) {
//      // Quick access to the data members
//      const AugmentedLaserScan& scan1 = scan1_iter->second;
//      const AugmentedLaserScan& scan2 = scan2_iter->second;
//
//      // Write a CSM log for debugging
//      std::ostringstream filename;
//      filename << debug_log_path << "/pair_" << std::fixed << std::setprecision(3) << timestamp1.toSec() << "_" << std::fixed << std::setprecision(3) << timestamp2.toSec() << ".csv";
//      std::ofstream logfile(filename.str().c_str());
//
//      logfile << timestamp1;
//      logfile << " " << scan1.map_pose.x() << " " << scan1.map_pose.y() << " " << scan1.map_pose.theta();
//      for(size_t m = 0; m < scan1.cloud.height; ++m) {
//        for(size_t n = 0; n < scan1.cloud.width; ++n) {
//          float x = *((float*)(&scan1.cloud.data[m*scan1.cloud.row_step + n*scan1.cloud.point_step + 0*sizeof(float)]));
//          float y = *((float*)(&scan1.cloud.data[m*scan1.cloud.row_step + n*scan1.cloud.point_step + 1*sizeof(float)]));
//          logfile << " " << x << " " << y;
//        }
//      }
//      logfile << std::endl;
//
//      logfile << timestamp2;
//      logfile << " " << scan2.map_pose.x() << " " << scan2.map_pose.y() << " " << scan2.map_pose.theta();
//      for(size_t m = 0; m < scan2.cloud.height; ++m) {
//        for(size_t n = 0; n < scan2.cloud.width; ++n) {
//          float x = *((float*)(&scan2.cloud.data[m*scan2.cloud.row_step + n*scan2.cloud.point_step + 0*sizeof(float)]));
//          float y = *((float*)(&scan2.cloud.data[m*scan2.cloud.row_step + n*scan2.cloud.point_step + 1*sizeof(float)]));
//          logfile << " " << x << " " << y;
//        }
//      }
//      logfile << std::endl;
//
//      // Close the logfile
//      logfile.close();
//    }
//
//    // Print a progress message
//    printProgressBar("  Writing LaserScan Pairs Log: ", 100 * (double)(pair_counter++) / pairs.size());
//  }
//
//  // Add the results to the output
//  timer.stop();
//  std::cout << ", LaserScan Pairs: " << pairs.size() << ", Time: " << timer.elapsed() << std::endl;
//}
//
///* ************************************************************************* */
//void WriteLaserscanMatches(const RelativePoseEstimates& matches, const std::string& filename) {
//
//  Timer timer;
//  timer.start();
//
//  // create an output file
//  boost::filesystem::path file_path(filename);
//  boost::filesystem::create_directories(file_path.parent_path());
//  std::ofstream logfile(filename.c_str());
//
//  // Loop through each scan, writing the output to disk
//  size_t match_counter = 0;
//  for(size_t i = 0; i < matches.size(); ++i) {
//    const RelativePoseEstimate& match = matches[i];
//
//    logfile << match.timestamp1 << " " << match.timestamp2;
//    logfile << " " << match.relative_pose.x() << " " << match.relative_pose.y() << " " << match.relative_pose.theta();
//    for(size_t m = 0; m < match.cov.rows(); ++m) {
//      for(size_t n = 0; n < match.cov.cols(); ++n) {
//        logfile << " " << match.cov(m,n);
//      }
//    }
//    logfile << std::endl;
//
//    // Print a progress message
//    printProgressBar("  Writing LaserScan Match Log: ", 100 * (double)(match_counter++) / matches.size());
//  }
//
//  // Close the logfile
//  logfile.close();
//
//  // Add the results to the output
//  timer.stop();
//  std::cout << ", Matches: " << matches.size() << ", Time: " << timer.elapsed() << std::endl;
//}
//
///* ************************************************************************* */
//void WriteLaserscanMatchExtended(const sensor_msgs::LaserScan& scan1, const sensor_msgs::LaserScan& scan2, gtsam::Pose2& initial_guess, gtsam::Pose2& final_pose, const std::string& filename) {
//  // create an output directory
//  boost::filesystem::path file_path(filename);
//  boost::filesystem::create_directories(file_path.parent_path());
//
//  // Project laserscan data into (x,y)
//  laser_geometry::LaserProjection projector;
//  sensor_msgs::PointCloud2 cloud1;
//  projector.projectLaser(scan1, cloud1);
//  sensor_msgs::PointCloud2 cloud2;
//  projector.projectLaser(scan2, cloud2);
//
//  // Write a log for debugging
//  std::ofstream logfile(filename.c_str());
//
//  logfile << 0.0;
//  logfile << " " << 0.0 << " " << 0.0 << " " << 0.0;
//  for(size_t m = 0; m < cloud1.height; ++m) {
//    for(size_t n = 0; n < cloud1.width; ++n) {
//      float x = *((float*)(&cloud1.data[m*cloud1.row_step + n*cloud1.point_step + 0*sizeof(float)]));
//      float y = *((float*)(&cloud1.data[m*cloud1.row_step + n*cloud1.point_step + 1*sizeof(float)]));
//      logfile << " " << x << " " << y;
//    }
//  }
//  logfile << std::endl;
//
//  logfile << 1.0;
//  logfile << " " << initial_guess.x() << " " << initial_guess.y() << " " << initial_guess.theta();
//  for(size_t m = 0; m < cloud2.height; ++m) {
//    for(size_t n = 0; n < cloud2.width; ++n) {
//      float x = *((float*)(&cloud2.data[m*cloud2.row_step + n*cloud2.point_step + 0*sizeof(float)]));
//      float y = *((float*)(&cloud2.data[m*cloud2.row_step + n*cloud2.point_step + 1*sizeof(float)]));
//      logfile << " " << x << " " << y;
//    }
//  }
//  logfile << std::endl;
//
//  logfile << 1.0;
//  logfile << " " << final_pose.x() << " " << final_pose.y() << " " << final_pose.theta();
//  for(size_t m = 0; m < cloud2.height; ++m) {
//    for(size_t n = 0; n < cloud2.width; ++n) {
//      float x = *((float*)(&cloud2.data[m*cloud2.row_step + n*cloud2.point_step + 0*sizeof(float)]));
//      float y = *((float*)(&cloud2.data[m*cloud2.row_step + n*cloud2.point_step + 1*sizeof(float)]));
//      logfile << " " << x << " " << y;
//    }
//  }
//  logfile << std::endl;
//
//  // Close the logfile
//  logfile.close();
//}
//
///* ************************************************************************* */
//void WriteLaserscanMatchesExtended(const AugmentedScans& augmented_scans, const RelativePoseEstimates& matches, const std::string& debug_log_path) {
//
//  Timer timer;
//  timer.start();
//
//  // create an output directory
//  boost::filesystem::path file_path(debug_log_path);
//  boost::filesystem::create_directories(file_path);
//
//  // Print a progress message
//  printProgressBar("  Writing Extended LaserScan Matches Log: ", 0);
//
//  // Loop through each scan, writing the output to disk
//  size_t match_counter = 0;
//  for(RelativePoseEstimates::const_iterator iter = matches.begin(); iter != matches.end(); ++iter) {
//    // Get the current timestamp
//    ros::Time timestamp1 = iter->timestamp1;
//    ros::Time timestamp2 = iter->timestamp2;
//
//    // Look up the augmented scan for each timestamp
//    AugmentedScans::const_iterator scan1_iter = augmented_scans.find(timestamp1);
//    AugmentedScans::const_iterator scan2_iter = augmented_scans.find(timestamp2);
//
//    // Check that all required information was found
//    if(    (scan1_iter != augmented_scans.end())
//        && (scan2_iter != augmented_scans.end())
//    ) {
//      // Quick access to the data members
//      const AugmentedLaserScan& scan1 = scan1_iter->second;
//      const AugmentedLaserScan& scan2 = scan2_iter->second;
//
//      // Write a log for debugging
//      std::ostringstream filename;
//      filename << debug_log_path << "/match_" << std::fixed << std::setprecision(3) << timestamp1.toSec() << "_" << std::fixed << std::setprecision(3) << timestamp2.toSec() << ".csv";
//      std::ofstream logfile(filename.str().c_str());
//
//      logfile << timestamp1;
//      logfile << " " << scan1.map_pose.x() << " " << scan1.map_pose.y() << " " << scan1.map_pose.theta();
//      for(size_t m = 0; m < scan1.cloud.height; ++m) {
//        for(size_t n = 0; n < scan1.cloud.width; ++n) {
//          float x = *((float*)(&scan1.cloud.data[m*scan1.cloud.row_step + n*scan1.cloud.point_step + 0*sizeof(float)]));
//          float y = *((float*)(&scan1.cloud.data[m*scan1.cloud.row_step + n*scan1.cloud.point_step + 1*sizeof(float)]));
//          logfile << " " << x << " " << y;
//        }
//      }
//      logfile << std::endl;
//
//      logfile << timestamp2;
//      logfile << " " << scan2.map_pose.x() << " " << scan2.map_pose.y() << " " << scan2.map_pose.theta();
//      for(size_t m = 0; m < scan2.cloud.height; ++m) {
//        for(size_t n = 0; n < scan2.cloud.width; ++n) {
//          float x = *((float*)(&scan2.cloud.data[m*scan2.cloud.row_step + n*scan2.cloud.point_step + 0*sizeof(float)]));
//          float y = *((float*)(&scan2.cloud.data[m*scan2.cloud.row_step + n*scan2.cloud.point_step + 1*sizeof(float)]));
//          logfile << " " << x << " " << y;
//        }
//      }
//      logfile << std::endl;
//
//      gtsam::Pose2 pose3 = scan1.map_pose.compose(iter->relative_pose);
//      logfile << timestamp2;
//      logfile << " " << pose3.x() << " " << pose3.y() << " " << pose3.theta();
//      for(size_t m = 0; m < scan2.cloud.height; ++m) {
//        for(size_t n = 0; n < scan2.cloud.width; ++n) {
//          float x = *((float*)(&scan2.cloud.data[m*scan2.cloud.row_step + n*scan2.cloud.point_step + 0*sizeof(float)]));
//          float y = *((float*)(&scan2.cloud.data[m*scan2.cloud.row_step + n*scan2.cloud.point_step + 1*sizeof(float)]));
//          logfile << " " << x << " " << y;
//        }
//      }
//      logfile << std::endl;
//
//      // Close the logfile
//      logfile.close();
//    }
//
//    // Print a progress message
//    printProgressBar("  Writing LaserScan Matches Log: ", 100 * (double)(match_counter++) / matches.size());
//  }
//
//  // Add the results to the output
//  timer.stop();
//  std::cout << ", LaserScan Matches: " << matches.size() << ", Time: " << timer.elapsed() << std::endl;
//}
//
///* ************************************************************************* */
//void WriteLaserscanMatchesPly(const Poses& poses, const RelativePoseEstimates& matches, const std::string& filename) {
//
//  Timer timer;
//  timer.start();
//
//  // create an output file
//  boost::filesystem::path file_path(filename);
//  boost::filesystem::create_directories(file_path.parent_path());
//  std::ofstream logfile(filename.c_str(), std::ios_base::binary);
//
//  // Write PLY file header
//  logfile << "ply" << std::endl;
//  logfile << "format binary_little_endian 1.0" << std::endl;
//  logfile << "element vertex " << poses.size() << std::endl;
//  logfile << "property float x" << std::endl;
//  logfile << "property float y" << std::endl;
//  logfile << "property float z" << std::endl;
//  logfile << "property int index" << std::endl;
//  logfile << "property float timestamp" << std::endl;
//  logfile << "element face " << matches.size() << std::endl;
//  logfile << "property list uchar int vertex_index" << std::endl;
//  logfile << "end_header" << std::endl;
//
//  // Write the path
//  int vertex_count = 0;
//  for(Poses::const_iterator poses_iter = poses.begin(); poses_iter != poses.end(); ++poses_iter) {
//    float x = poses_iter->second.x();
//    float y = poses_iter->second.y();
//    float z = 0.0;
//    float time = poses_iter->first.toSec();
//
//    logfile.write((char*)&x, sizeof(x));
//    logfile.write((char*)&y, sizeof(y));
//    logfile.write((char*)&z, sizeof(z));
//    logfile.write((char*)&vertex_count, sizeof(vertex_count));
//    logfile.write((char*)&time, sizeof(time));
//
//    vertex_count++;
//  }
//
//
//  // Write the loop closure edges (as faces)
//  int edge_count = 0;
//  for(RelativePoseEstimates::const_iterator matches_iter = matches.begin(); matches_iter != matches.end(); ++matches_iter) {
//    unsigned char vertex_count = 3;
//    int index1 = std::distance(poses.begin(), poses.find(matches_iter->timestamp1));
//    int index2 = std::distance(poses.begin(), poses.find(matches_iter->timestamp2));
//    int index3 = index1;
//
//    logfile.write((char*)&vertex_count, sizeof(vertex_count));
//    logfile.write((char*)&index1, sizeof(index1));
//    logfile.write((char*)&index2, sizeof(index2));
//    logfile.write((char*)&index3, sizeof(index3));
//
//    edge_count++;
//  }
//
//  // Close the logfile
//  logfile.close();
//
//  // Add the results to the output
//  timer.stop();
//}

/* ************************************************************************* */
} /// @namespace laserscan

} /// @namespace bnr_mapping


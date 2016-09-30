/**
 * odometry_processing.cpp
 * This initial implementation may consume more memory than is necessary.
 * If memory usage is an issue, the processing can be reformulated such that
 * data is processed as it is extracted, instead of extracting all
 * data, then processing.
 */

#include <aslam_demo/mapping/odometry_processing.h>
//#include <mapping/factorgraph_processing.h>
#include <aslam_demo/mapping/timer.h>
#include <aslam_demo/factors/odometry_factor.h>
#include <aslam_demo/factors/key_generator.h>

namespace mapping {

namespace odometry {

/* ************************************************************************* */
/*Timestamps sparsifyOdometry(const Odometry& odometry, double time_tolerance, double time_delta_threshold, double distance_delta_threshold, double rotation_delta_threshold) {
  Timestamps timestamps;

  Timer timer;
  timer.start();

  // Check that some odometry messages exist
  if(odometry.empty()) {
    throw std::runtime_error("No odometry measurements provided.");
  }

  // Create a key generator for timestamp lookups
  factors::KeyGenerator key_generator(time_tolerance);

  // Create accumulator variables
  double time_delta = 0.0;
  double distance_delta = 0.0;
  double rotation_delta = 0.0;

  // Loop over the messages, extracting sparse timestamps at the configured intervals
  Odometry::const_iterator previous_iter;
  Odometry::const_iterator current_iter;
  size_t counter = 0;
  for(Odometry::const_iterator iter = odometry.begin(); iter != odometry.end(); ++iter) {

    // Update the previous and current iterators
    previous_iter = current_iter;
    current_iter = iter;

    // Odometry sparsification operates on relative information. Skip the first entry.
    if(current_iter == odometry.begin()) continue;

    // Get a reference to the previous and current odometry entries
    const ros::Time& previous_time = previous_iter->first;
    const nav_msgs::Odometry& previous_message = previous_iter->second;
    const ros::Time& current_time = current_iter->first;
    const nav_msgs::Odometry& current_message = current_iter->second;

    // Extract the 2D poses from the messages
    gtsam::Pose2 previous_pose(previous_message.pose.pose.position.x, previous_message.pose.pose.position.y,
        tf::getYaw(tf::Quaternion(previous_message.pose.pose.orientation.x, previous_message.pose.pose.orientation.y, previous_message.pose.pose.orientation.z, previous_message.pose.pose.orientation.w))
    );
    gtsam::Pose2 current_pose(current_message.pose.pose.position.x, current_message.pose.pose.position.y,
        tf::getYaw(tf::Quaternion(current_message.pose.pose.orientation.x, current_message.pose.pose.orientation.y, current_message.pose.pose.orientation.z, current_message.pose.pose.orientation.w))
    );

    // Compute the pose delta
    gtsam::Pose2 delta = previous_pose.between(current_pose);

    // Accumulate the time, distance, rotation change since the last selected pose
    time_delta += (current_time - previous_time).toSec();
    distance_delta += delta.t().vector().norm();
    rotation_delta += std::fabs(delta.theta());

    // If any of the computed deltas exceed the configured thresholds, store the timestamp
    if(  (time_delta >= time_delta_threshold)
      || (distance_delta >= distance_delta_threshold)
      || (rotation_delta >= rotation_delta_threshold) ) {
      // Quantize the timestamp using the key generator
      ros::Time quantized_time = key_generator.extractTimestamp(key_generator.generateKey(factors::key_type::Pose2, current_time));
      // Select this timestamp
      timestamps.insert(quantized_time);
      // Clear the accumulators
      time_delta = 0.0;
      distance_delta = 0.0;
      rotation_delta = 0.0;
    }
  }

  // Add the results to the output
  timer.stop();
  ROS_DEBUG_STREAM("Sparsified " << odometry.size() << " nav_msgs::Odometry messages into " << timestamps.size() << " timestamps in " << timer.elapsed() << " seconds.");

  return timestamps;
}
*/
/* ************************************************************************* */
/*RelativePoseEstimates computeRelativePoses(const Odometry& odometry, const Timestamps& timestamps, const gtsam::Vector& sigmas, double time_tolerance, double scale) {
  RelativePoseEstimates relative_poses;

  Timer timer;
  timer.start();

  // Create a key generator for timestamp lookups
  factors::KeyGenerator key_generator(time_tolerance);

  // Accumulate odometry measurements between the provided timestamps
  // If an odometry message spans the requested timestamps, it must be split into two pieces
  // The timestamps are intrinsically sorted; we will assume the odometry messages are also in time order

  // Define the matrix G, used to calculate the covariance
  gtsam::Matrix G(3,4);
  G(0,0) = sigmas(0); G(0,1) = 0.0;       G(0,2) = sigmas(1); G(0,3) = sigmas(2);
  G(1,0) = 0.0;       G(1,1) = sigmas(0); G(1,2) = sigmas(1); G(1,3) = sigmas(2);
  G(2,0) = sigmas(3); G(2,1) = sigmas(3); G(2,2) = sigmas(4); G(2,3) = sigmas(5);

  // Loop over all of the provided timestamps, computing relative pose information between each sequential pair
  Timestamps::const_iterator timestamp1_iter;
  Timestamps::const_iterator timestamp2_iter;
  for(Timestamps::const_iterator timestamp_iter = timestamps.begin(); timestamp_iter != timestamps.end(); ++timestamp_iter) {

    // Update the previous and current timestamp iterator references
    timestamp1_iter = timestamp2_iter;
    timestamp2_iter = timestamp_iter;

    // Relative poses are calculated between timestamps. Skip the first entry.
    if(timestamp2_iter == timestamps.begin()) continue;

    // Quick access to the timestamps
    const ros::Time& timestamp1 = *timestamp1_iter;
    const ros::Time& timestamp2 = *timestamp2_iter;

    // Find the range of odometry entries needed to compute the desired relative pose. The odometry change is computed between
    // two consecutive odometry messages. For C++ design reasons, it is easier to find the older odometry entry of the pair; the
    // younger entry is simply computed as entry before.
    // Because the mapping system timestamps are quantized, an additional check is needed to catch situations where the raw odometry
    // timestamp is older than the target, but the quantized timestamp is equal.
    Odometry::const_iterator odometry2_begin = odometry.upper_bound(timestamp1); if(key_generator.computeQuantizedTimestamp(odometry2_begin->first) <= timestamp1) ++odometry2_begin;
    Odometry::const_iterator odometry2_end = odometry.lower_bound(timestamp2);   if(odometry2_end != odometry.end()) ++odometry2_end;

    // Create a new relative pose structure
    RelativePoseEstimate relative_pose;
    relative_pose.timestamp1 = timestamp1;
    relative_pose.timestamp2 = timestamp2;
    relative_pose.relative_pose = gtsam::Pose2::identity();
    relative_pose.cov = gtsam::Matrix::Zero(3,3);

    // Loop over the incremental odometry information, accumulating the relative pose and covariance
    for(Odometry::const_iterator odometry2_iter = odometry2_begin; odometry2_iter != odometry2_end; ++odometry2_iter) {
      // Get an iterator pointing to the previous entry
      Odometry::const_iterator odometry1_iter = odometry2_iter; --odometry1_iter;

      // Quantize the odometry timestamps
      ros::Time odometry_timestamp1 = key_generator.computeQuantizedTimestamp(odometry1_iter->first);
      ros::Time odometry_timestamp2 = key_generator.computeQuantizedTimestamp(odometry2_iter->first);

      // Compute the time range that must be extracted from this odometry pair
      ros::Time pose_increment_timestamp1 = std::max(timestamp1, odometry1_iter->first);
      ros::Time pose_increment_timestamp2 = std::min(timestamp2, odometry2_iter->first);

      // Extract the relative pose
      gtsam::Pose2 pose_increment = splitOdometry(odometry1_iter->second, odometry2_iter->second, pose_increment_timestamp1, pose_increment_timestamp2);
      double time_increment = (pose_increment_timestamp2 - pose_increment_timestamp1).toSec();

      // Apply any requested scale correction
      pose_increment = gtsam::Pose2(pose_increment.x()*scale, pose_increment.y()*scale, pose_increment.theta());

      // Accumulate the pose
      gtsam::Matrix H2;
      relative_pose.relative_pose = relative_pose.relative_pose.compose(pose_increment, boost::none, H2);

      // Accumulate the covariance
      gtsam::Vector delta(4);
      delta(0) = pose_increment.x();
      delta(1) = pose_increment.y();
      delta(2) = pose_increment.theta();
      delta(3) = time_increment;
      relative_pose.cov = H2 * relative_pose.cov * H2.transpose() + (G * delta)*(G * delta).transpose();
    }

    // Insert the computed relative pose into the results
    relative_poses.push_back(relative_pose);
  }

  // Add the results to the output
  timer.stop();
  ROS_DEBUG_STREAM("Computed " << relative_poses.size() << " relative poses from nav_msgs::Odometry messages in " << timer.elapsed() << " seconds.");

  return relative_poses;
}
/*
/* ************************************************************************* */
gtsam::Pose2 splitOdometry(const nav_msgs::Odometry& previous_message, const nav_msgs::Odometry& next_message, const ros::Time& timestamp1, const ros::Time& timestamp2) {
  gtsam::Pose2 relative_pose;

  // Convert the odometry data into GTSAM poses
  gtsam::Pose2 pose1(previous_message.pose.pose.position.x, previous_message.pose.pose.position.y,
      tf::getYaw(tf::Quaternion(previous_message.pose.pose.orientation.x, previous_message.pose.pose.orientation.y, previous_message.pose.pose.orientation.z, previous_message.pose.pose.orientation.w)));
  gtsam::Pose2 pose4(next_message.pose.pose.position.x, next_message.pose.pose.position.y,
      tf::getYaw(tf::Quaternion(next_message.pose.pose.orientation.x, next_message.pose.pose.orientation.y, next_message.pose.pose.orientation.z, next_message.pose.pose.orientation.w)));

  // (1) Compute the absolute pose (pose2) between (previous_message, timestamp1)
  gtsam::Vector pose_delta_14 = pose1.localCoordinates(pose4);
  double time_delta_14 = (next_message.header.stamp - previous_message.header.stamp).toSec();
  double time_delta_12 = (timestamp1 - previous_message.header.stamp).toSec();
  gtsam::Vector pose_delta_12 = (time_delta_12/time_delta_14)*pose_delta_14;
  gtsam::Pose2 pose2 = pose1.retract(pose_delta_12);

  // (2) Compute the absolute pose (pose3) between (timestamp2, next_message)
  double time_delta_13 = (timestamp2 - previous_message.header.stamp).toSec();
  gtsam::Vector pose_delta_13 = (time_delta_13/time_delta_14)*pose_delta_14;
  gtsam::Pose2 pose3 = pose1.retract(pose_delta_13);

  // (3) Compute the relative pose between pose2 and pose3
  relative_pose = pose2.between(pose3);

  return relative_pose;
}

/* ************************************************************************* */
/*gtsam::NonlinearFactorGraph createOdometryFactors(const RelativePoseEstimates& relative_poses, double time_tolerance) {
  gtsam::NonlinearFactorGraph factors;

  Timer timer;
  timer.start();

  if(relative_poses.empty()) {
    throw std::runtime_error("No relative poses available for factor creation.");
  }

  // Create a GTSAM Key Generator for creating variable names
  factors::KeyGenerator key_generator(time_tolerance);

  // Loop over all of the odometry data, creating a factor for each relative pose
  for(size_t i = 0; i < relative_poses.size(); ++i) {

    // (1) Create a noise model based on the previously calculated covariance matrix
    gtsam::noiseModel::Base::shared_ptr noise_model = gtsam::noiseModel::Gaussian::Covariance(relative_poses[i].cov, true);

    // (2) Create a factor from the relative pose and cov
    gtsam::Key key1 = key_generator.generateKey(factors::key_type::Pose2, relative_poses[i].timestamp1);
    gtsam::Key key2 = key_generator.generateKey(factors::key_type::Pose2, relative_poses[i].timestamp2);
    gtsam::NonlinearFactor::shared_ptr factor(new factors::OdometryFactor(key1, key2, relative_poses[i].relative_pose, noise_model));
    factors.push_back(factor);
  }

  // Add the results to the output
  timer.stop();
  ROS_DEBUG_STREAM("Computed " << factors.size() << " odometry factors in " << timer.elapsed() << " seconds.");

  return factors;
}
*/
/* ************************************************************************* */
} /// @namespace odometry

} /// @namespace mapping


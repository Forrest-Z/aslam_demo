/**
 * odometry_processing.h
 */

#ifndef ODOMETRY_PROCESSING_H
#define ODOMETRY_PROCESSING_H

#include <aslam_demo/mapping/mapping_common.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <string>
#include <vector>

namespace mapping {

namespace odometry {

/**
 * Extract timestamps from the odometry that occur at configured time,
 * distance, or rotation intervals
 * @param configuration The set of configuration parameters
 * @return A sorted container of unique timestamps for desired odometry poses
 */
//Timestamps sparsifyOdometry(const Odometry& odometry, double time_tolerance, double time_delta_threshold, double distance_delta_threshold, double rotation_delta_threshold);

/**
 * Compute the relative pose and covariance between each provided timestamp
 * @param odometry
 * @param timestamps
 * @param sigmas
 * @param time_tolerance
 * @return A collection of relative pose information
 */
RelativePoseEstimates computeRelativePoses(const Odometry& odometry, const Timestamps& timestamps, const gtsam::Vector& sigmas, double time_tolerance, double scale = 1.0);

/**
 * Compute relative poses between the previous message time and the target time, and between the target time and next message time
 * @param previous_message
 * @param next_message
 * @param target_timestamp
 * @return Two relative poses
 */
gtsam::Pose2 splitOdometry(const nav_msgs::Odometry& previous_message, const nav_msgs::Odometry& next_message, const ros::Time& timestamp1, const ros::Time& timestamp2);

/**
 * Process the provided odometry data to produce a set of
 * GTSAM odometry factors and initial conditions
 * @param relative_poses
 * @param time_tolerance
 * @return A GTSAM factor graph of odometry factors
 */
gtsam::NonlinearFactorGraph createOdometryFactors(const RelativePoseEstimates& relative_poses, double time_tolerance);

} /// @namespace odometry

} /// @namespace mapping

#endif // ODOMETRY_PROCESSING_H



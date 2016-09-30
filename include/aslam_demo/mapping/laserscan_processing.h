/**
 * laserscan_processing.h
 */

#ifndef LASERSCAN_PROCESSING_H
#define LASERSCAN_PROCESSING_H

#include <aslam_demo/mapping/mapping_common.h>
#include <ros/ros.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/Matrix.h>
#include <string>
#include <vector>

namespace mapping {

namespace laserscan {

/**
 * Based on the configuration parameters, remove scans between the time/distance/rotation deltas
 * @param scans
 */
LaserScans sparsifyLaserScans(const LaserScans& scans, const Poses& poses, double time_delta_threshold, double distance_delta_threshold, double rotation_delta_threshold);

/**
 * Read the scan filter configuration file, parse out the YAML, and convert it to XmlRpc.
 * @param filename Filename of a scan filter YAML configuration file
 * @return The ROS XmlRpc version of the YAML
 */
XmlRpc::XmlRpcValue readFilterConfiguration(const std::string& filename);

/**
 * Apply some laser scan filters to the source scan before matching
 * @param scans
 */
LaserScans filterLaserScans(const LaserScans& scans, const std::string& filename);

/**
 * Convert the laser scan into a point cloud using the tf data
 * @param scans
 * @param transforms
 */
PointClouds computePointClouds(const LaserScans& scans, const TfTransforms& transforms, const std::string& target_frame);

/**
 *
 * @param odom_poses
 * @param map_poses
 * @param scans
 * @param clouds
 * @return
 */
AugmentedLaserScans createAugmentedLaserScans(const Poses& odom_poses, const Poses& map_poses, const LaserScans& scans, const PointClouds& clouds);

///**
// *
// * @param pose1
// * @param pose2
// * @param angle_min
// * @param angle_max
// * @param range_max
// * @return
// */
//double computeOverlap(const gtsam::Pose2& pose1, const gtsam::Pose2& pose2, double angle_min, double angle_max, double range_max);
//
///**
// *
// * @param pose1
// * @param pose2
// * @param scan1
// * @param scan2
// * @return
// */
//double computeOverlap(const gtsam::Pose2& pose1, const gtsam::Pose2& pose2, const sensor_msgs::LaserScan& scan1, const sensor_msgs::LaserScan& scan2);

/**
 * Compute the set of laserscan pairs to match based on changing keyframes according to the provided deltas
 * @param poses
 * @param time_delta_threshold
 * @param distance_delta_threshold
 * @param rotation_delta_threshold
 * @return
 */
TimestampPairs computeAugmentedLaserScanPairs(const AugmentedLaserScans& augmented_scans, double time_delta_threshold, double distance_delta_threshold, double rotation_delta_threshold);

/**
 * Compute the set of laserscan pairs from the reference pose set. The matches are based on a set of "corrected" poses. This function searches for
 * poses in the reference set that are near (within maximum_distance_delta meters) the corrected pose and are older (by at least minimum_time_delta
 * seconds). The second laserscan in the pair is scan closest to the corrected pose timestamp within the reference set. This is intended for loop
 * closure operations.
 * @param reference_poses
 * @param corrected_poses
 * @param minimum_time_delta
 * @param maximum_time_delta
 * @param minimum_distance_delta
 * @param maximum_distance_delta
 * @param minimum_rotation_delta
 * @param maximum_rotation_delta
 * @return
 */
TimestampPairs computeAugmentedLaserScanPairs(const AugmentedLaserScans& augmented_scans,
    double minimum_time_delta, double maximum_time_delta,
    double minimum_x_delta, double maximum_x_delta,
    double minimum_y_delta, double maximum_y_delta,
    double minimum_rotation_delta, double maximum_rotation_delta);

/**
 * Process the provided relative pose estimates to produce a set of
 * GTSAM factors
 * @param configuration The set of configuration parameters
 * @param scans Vector of ROS LaserScan messages
 * @return A GTSAM factor graph of laser scan factors,
 * and a GTSAM Values structure with initial guesses for each variable
 */
gtsam::NonlinearFactorGraph createLaserScanFactors(const RelativePoseEstimates& matches, double time_tolerance);


///**
// * Process the provided  relative pose estimates to produce a set of
// * initial value guesses for use as starting conditions in an optimization
// * @param configuration The set of configuration parameters
// * @param scans Vector of ROS LaserScan messages
// * @return A GTSAM factor graph of laser scan factors,
// * and a GTSAM Values structure with initial guesses for each variable
// */
//gtsam::Values createLaserscanValues(const RelativePoseEstimates& matches, const gtsam::Pose2& initial_pose, double time_tolerance);

///**
// * Serialize the extracted scans to a log file for easy MATLAB parsing.
// * Format: Timestamp Pose(e.g. x y theta) Scan Returns (1081 x,y pairs)
// * @param scans The set of scans to serialize
// */
//void writeLaserscanPoints(const Scans& scans, const Clouds& clouds, const Poses& poses, const std::string& filename);
//
///**
// * Serialize the extracted loop closure pairs to a log file for easy MATLAB parsing
// * @param pairs
// * @param augmented_scans
// * @param debug_log_path
// */
//void writeLaserscanPairs(const TimestampPairs& pairs, const AugmentedScans& augmented_scans, const std::string& debug_log_path);
//
///**
// * Serialize the extracted scans to a log file for easy MATLAB parsing.
// * Format: Timestamp Pose(e.g. x y theta) Scan Returns (1081 x,y pairs)
// * @param scans The set of scans to serialize
// */
//void writeLaserscanMatches(const RelativePoseEstimates& matches, const std::string& filename);
//
///**
// * Serialize the extracted scans to a log file for easy MATLAB parsing.
// * Format: Timestamp Pose(e.g. x y theta) Scan Returns (1081 x,y pairs)
// * @param scans The set of scans to serialize
// */
//void writeLaserscanMatchExtended(const sensor_msgs::LaserScan& scan1, const sensor_msgs::LaserScan& scan2, gtsam::Pose2& initial_guess, gtsam::Pose2& final_pose, const std::string& filename);
//
///**
// * Serialize the extracted scans to a log file for easy MATLAB parsing.
// * Format: Timestamp Pose(e.g. x y theta) Scan Returns (1081 x,y pairs)
// * @param scans The set of scans to serialize
// */
//void writeLaserscanMatchesExtended(const AugmentedScans& augmented_scans, const RelativePoseEstimates& matches, const std::string& debug_log_path);
//
///**
// * Serialize the extracted scans to a PLY file for easy viewing.
// * @param scans The set of scans to serialize
// */
//void writeLaserscanMatchesPly(const Poses& poses, const RelativePoseEstimates& matches, const std::string& filename);

} /// @namespace laserscan_processing

} /// @namespace bnr_mapping

#endif // LASERSCAN_PROCESSING_H


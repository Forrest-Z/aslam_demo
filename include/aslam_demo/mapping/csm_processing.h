/**
 * csm_processing.h
 */

#ifndef CSM_PROCESSING_H
#define CSM_PROCESSING_H

#include <aslam_demo/mapping/laserscan_processing.h>
#include <aslam_demo/mapping/mapping_common.h>
#include <csm/csm.h>
#include <ros/ros.h>
#include <gtsam/geometry/Pose3.h>
#include <string>
#include <fstream>
#include <limits.h>

namespace mapping {

namespace csm {


/**
 * Use the CSM library to compute relative poses between scans
 * @param scan1 ROS laserscan message from timestamp1
 * @param scan2 ROS laserscan message from timestamp2
 * @param csm_params The CSM configuration parameters to use during scan matching
 * @param initial_guess An initial guess of the relative pose from timestamp1 to timestamp2
 * @param base_T_laser An optional frame transformation from the sensor frame to the robot frame
 * @param laser_sigma An optional measurement error estimate for the laser data
 * @param covariance_trace_threshold An outlier detection threshold. All matches that have a covaraince trace larger than this threshold will report as failed.
 * @param initial_guess_error_threshold An outlier detection threshold. All matches with a euclidean distance between the initial guess and the final match greater than the threshold will report as failed.
 * @param csm_filename An optional filename. When provided, a CSM log file will be generated.
 * @return The relative pose and covariance based on laser scan matching
 */
RelativePoseEstimate computeLaserScanMatch(const sensor_msgs::LaserScan& scan1,
    const sensor_msgs::LaserScan& scan2,
    struct sm_params& csm_params,
    const gtsam::Pose2& initial_guess,
    const gtsam::Pose3& base_T_laser = gtsam::Pose3::identity(),
    double laserscan_sigma = 0.05,
    double covariance_trace_threshold = 10000000000000000,
    double initial_guess_error_threshold = 100000000000000000,
    const std::string& csm_filename = "");

/**
 * Use the CSM library to compute relative poses between scans
 * @param scans ROS laserscan messages. Scann matches will be computed between consecutive entries in the vector (1-2, 2-3, 3-4, ...)
 * @param csm_params The CSM configuration parameters to use during scan matching
 * @param initial_guesses An optional initial guess of the absolute poses for each timestamp. The identity transform is assumed if not provided.
 * @param base_T_laser An optional frame transformation from the sensor frame to the robot frame
 * @param laser_sigma An optional measurement error estimate for the laser data
 * @param covariance_trace_threshold An outlier detection threshold. All matches that have a covaraince trace larger than this threshold will report as failed.
 * @param initial_guess_error_threshold An outlier detection threshold. All matches with a euclidean distance between the initial guess and the final match greater than the threshold will report as failed.
 * @param csm_path An optional path. When provided, a CSM log file will be generated for each laserscan pair.
 * @return The relative pose and covariance based on laser scan matching
 */
/*RelativePoseEstimates computeLaserScanMatches(const std::vector<sensor_msgs::LaserScan>& scans,
    struct sm_params& csm_params,
    const std::vector<gtsam::Pose2>& initial_guesses = std::vector<gtsam::Pose2>(),
    const gtsam::Pose3& base_T_laser = gtsam::Pose3::identity(),
    double laserscan_sigma = 0.05,
    double covariance_trace_threshold = std::numeric_limits<double>::max(),
    double initial_guess_error_threshold = std::numeric_limits<double>::max(),
    const std::string& csm_path = "");*/

/**
 * Use the CSM library to compute relative poses between scans
 * @param scans1 ROS laserscan messages used as the reference scan (r1-t1, r2-t2, r3-t3, ...)
 * @param scans2 ROS laserscan messages used as the target scan (r1-t1, r2-t2, r3-t3, ...)
 * @param csm_params The CSM configuration parameters to use during scan matching
 * @param initial_guesses An optional container of initial guess of the relative pose from reference to target. The identity transform is assumed if not provided.
 * @param base_T_laser An optional frame transformation from the sensor frame to the robot frame
 * @param laser_sigma An optional measurement error estimate for the laser data
 * @param covariance_trace_threshold An outlier detection threshold. All matches that have a covaraince trace larger than this threshold will report as failed.
 * @param initial_guess_error_threshold An outlier detection threshold. All matches with a euclidean distance between the initial guess and the final match greater than the threshold will report as failed.
 * @param csm_path An optional path. When provided, a CSM log file will be generated for each laserscan pair.
 * @return The relative pose and covariance based on laser scan matching
 */
/*RelativePoseEstimates computeLaserScanMatches(const std::vector<sensor_msgs::LaserScan>& scans1,
    const std::vector<sensor_msgs::LaserScan>& scans2,
    struct sm_params& csm_params,
    const std::vector<gtsam::Pose2>& initial_guesses = std::vector<gtsam::Pose2>(),
    const gtsam::Pose3& base_T_laser = gtsam::Pose3::identity(),
    double laserscan_sigma = 0.05,
    double covariance_trace_threshold = std::numeric_limits<double>::max(),
    double initial_guess_error_threshold = std::numeric_limits<double>::max(),
    const std::string& csm_path = "");*/

/**
 * Use the CSM library to compute relative poses between scans
 * @param pairs
 * @param scans
 * @param poses
 * @param csm_params
 * @param laserscan_sigma
 * @param base_T_laser
 * @param covariance_trace_threshold An outlier detection threshold. All matches that have a covaraince trace larger than this threshold will report as failed.
 * @param initial_guess_error_threshold An outlier detection threshold. All matches with a euclidean distance between the initial guess and the final match greater than the threshold will report as failed.
 * @param debug_log_path
 * @param use_map_frame
 * @return
 */
/*RelativePoseEstimates computeLaserScanMatches(
    const AugmentedLaserScans& augmented_scans,
    const TimestampPairs& pairs,
    struct sm_params& csm_params,
    const gtsam::Pose3& base_T_laser,
    double laserscan_sigma,
    double covariance_trace_threshold = std::numeric_limits<double>::max(),
    double initial_guess_error_threshold = std::numeric_limits<double>::max(),
    const std::string& debug_log_path = "",
    bool use_map_frame = false);*/

///**
// *
// * @param relative_pose
// * @param scan1
// * @param scan2
// * @param correspondence_distance
// * @return
// */
//size_t computeScanCorrespondences(const gtsam::Pose2 relative_pose, const sensor_msgs::LaserScan& scan1, const sensor_msgs::LaserScan& scan2, double correspondence_distance);

///**
// * Filter out incorrect scan matches based on scan-to-scan point correspondences
// * @param scans
// * @param relative_pose_estimates
// * @param correspondence_distance
// * @param minimum_correspondences
// * @param debug_log
// * @return
// */
//RelativePoseEstimates filterLowOverlapMatches(const AugmentedScans& scans, const RelativePoseEstimates& relative_pose_estimates, double correspondence_distance, size_t minimum_correspondences, const std::string& debug_log = "");

/**
 *
 * @param stream
 * @param scan
 */
void streamCsmScan(std::ofstream& stream, struct laser_data* scan);

/**
 * Write a CSM log consisting of a single scan match (for debugging purposes)
 */
void writeCsmLog(struct laser_data* reference, struct laser_data* scan, const std::string& filename);

} /// @namespace csm

} /// @namespace bnr_mapping

#endif // CSM_PROCESSING_H


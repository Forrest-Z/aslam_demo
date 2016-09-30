/**
 * csm_processing.cpp
 */

#include <aslam_demo/mapping/csm_processing.h>
#include <aslam_demo/mapping/timer.h>
#include <csm_ros/csm_ros.h>
extern "C" {
  #include <csm/icp/icp.h>
}
#include <ros/ros.h>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry.hpp>
#include <fstream>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace mapping {

namespace csm {

/* ************************************************************************* */
RelativePoseEstimate computeLaserScanMatch(
    const sensor_msgs::LaserScan& scan1,
    const sensor_msgs::LaserScan& scan2,
    struct sm_params& csm_params,
    const gtsam::Pose2& initial_pose,
    const gtsam::Pose3& base_T_laser,
    double laserscan_sigma,
    double covariance_trace_threshold ,
    double initial_guess_error_threshold ,
    const std::string& csm_filename)
{
  csm_params.use_point_to_line_distance = true;
  csm_params.use_corr_tricks = true;
  csm_params.max_iterations = 1000;
  csm_params.max_angular_correction_deg = 1.57 * 180.0 / M_PI;
  csm_params.max_linear_correction = 2.0;
  csm_params.max_correspondence_dist = 2.0;
  csm_params.sigma = .01;
  csm_params.epsilon_xy = 0.0001;
  csm_params.epsilon_theta = .0001;
  csm_params.outliers_maxPerc = .95;
  csm_params.outliers_adaptive_order = .7;
  csm_params.outliers_adaptive_mult = 2.0;
  csm_params.outliers_remove_doubles = true;
  csm_params.do_visibility_test = true;
  csm_params.do_alpha_test = 1;
  csm_params.do_alpha_test_thresholdDeg = .35 * 180.0 / M_PI;
  csm_params.clustering_threshold = .05;
  csm_params.orientation_neighbourhood = 3;
  csm_params.restart = false;
  csm_params.restart_threshold_mean_error = 0.01;
  csm_params.restart_dt = 0.01;
  csm_params.restart_dtheta = 0.025;
  csm_params.do_compute_covariance = true; // We need to covariance matrix for sensor fusion
  csm_params.use_ml_weights = false; // Use the computed alpha angle to weight the correspondences. Must compute the angle for this to work.
  csm_params.use_sigma_weights = false; // Use the "readings_sigma" field to weight the correspondences. If false, no weight is used. If all the weights are the same, this is identical to not weighting them.
  csm_params.debug_verify_tricks = false; // Do not run the debug check

  // Apply specific required parameters
  csm_params.do_compute_covariance = true; // We need to covariance matrix for sensor fusion
  csm_params.use_ml_weights = false; // Use the computed alpha angle to weight the correspondences. Must compute the angle for this to work.
  csm_params.use_sigma_weights = false; // Use the "readings_sigma" field to weight the correspondences. If false, no weight is used. If all the weights are the same, this is identical to not weighting them.
  csm_params.debug_verify_tricks = false; // Do not run the debug check
  ROS_INFO_STREAM("r1");

  // Set the laser transformation (and determine if it is inverted)
  double roll = base_T_laser.rotation().roll();
  double pitch = base_T_laser.rotation().pitch();
  bool laser_inverted = (roll > 3.0) || (roll < -3.0) || (pitch > 3.0) || (pitch < -3.0);
  csm_params.laser[0] = base_T_laser.x();
  csm_params.laser[1] = base_T_laser.y();
  csm_params.laser[2] = base_T_laser.rotation().yaw();
  ROS_INFO_STREAM("r2");
  // Transform the initial pose into laserscan coordinates
  gtsam::Pose2 first_guess;
  {
    gtsam::Pose3 map_T_base1 = gtsam::Pose3::identity();
    gtsam::Pose3 map_T_laser1 = map_T_base1*base_T_laser;
    gtsam::Pose3 map_T_base2 = gtsam::Pose3(gtsam::Rot3::Rz(initial_pose.theta()), gtsam::Point3(initial_pose.x(), initial_pose.y(), 0.0));
    gtsam::Pose3 map_T_laser2 = map_T_base2*base_T_laser;
    gtsam::Pose3 delta = map_T_laser1.between(map_T_laser2);
    first_guess = gtsam::Pose2(delta.translation().x(), delta.translation().y(), delta.rotation().yaw());
  }
  ROS_INFO_STREAM("r3");

  // Convert the ROS laserscan messages into CSM laser structures (Note: This allocates memory)
  /// @todo: Do I need the 'laser_inverted' flag since I'm doing the above 3D operations?
  csm_params.laser_ref  = csm_ros::toCsmLaserData(scan1, laserscan_sigma, laser_inverted);
  csm_params.laser_sens = csm_ros::toCsmLaserData(scan2, laserscan_sigma, laser_inverted);
  // Set the min and max allowed laser range
  csm_params.min_reading = std::min<double>(scan1.range_min, scan1.range_min);
  csm_params.max_reading = std::max<double>(scan2.range_max, scan2.range_max);
  // Calculate an initial guess based on odometry
  csm_params.first_guess[0] = first_guess.x();
  csm_params.first_guess[1] = first_guess.y();
  csm_params.first_guess[2] = first_guess.theta();
  // Also write the first guess into the odometry for debugging
  csm_params.laser_ref->odometry[0] = 0.0;
  csm_params.laser_ref->odometry[1] = 0.0;
  csm_params.laser_ref->odometry[2] = 0.0;
  csm_params.laser_sens->odometry[0] = csm_params.first_guess[0];
  csm_params.laser_sens->odometry[1] = csm_params.first_guess[1];
  csm_params.laser_sens->odometry[2] = csm_params.first_guess[2];

  // If requested, write a CSM log for debugging
  if(!csm_filename.empty()) {
    writeCsmLog(csm_params.laser_ref, csm_params.laser_sens, csm_filename);
  }
  ROS_INFO_STREAM("r4");
  ROS_INFO_STREAM(csm_params.laser_ref->estimate[0]);
  // Use CSM to do the scan matching
  sm_result output;
  sm_icp(&csm_params, &output);
  ROS_INFO_STREAM("r5");

  // Release allocated memory
  ld_free(csm_params.laser_ref);
  ld_free(csm_params.laser_sens);
  ROS_INFO_STREAM("r6");

  // Check if the match was successful
  if(!output.valid) {
    throw(std::runtime_error("CSM was unable to find a valid scan match from " + boost::lexical_cast<std::string>(scan1.header.stamp.toSec()) + " to " + boost::lexical_cast<std::string>(scan2.header.stamp.toSec())));
  }

  // Transform the scan match pose back to robot coordinates
  gtsam::Pose2 relative_pose;
  {
    gtsam::Pose3 map_T_laser1 = gtsam::Pose3::identity();
    gtsam::Pose3 map_T_base1 = map_T_laser1*(base_T_laser.inverse());
    gtsam::Pose3 map_T_laser2 = gtsam::Pose3(gtsam::Rot3::Rz(output.x[2]), gtsam::Point3(output.x[0], output.x[1], 0.0));
    gtsam::Pose3 map_T_base2 = map_T_laser2*(base_T_laser.inverse());
    gtsam::Pose3 delta = map_T_base1.between(map_T_base2);
    relative_pose = gtsam::Pose2(delta.translation().x(), delta.translation().y(), delta.rotation().yaw());
  }
  ROS_INFO_STREAM("r1");

  // Create the output object
  RelativePoseEstimate match;
  match.timestamp1 = scan1.header.stamp;
  match.timestamp2 = scan2.header.stamp;
  match.relative_pose = relative_pose;
  match.cov = gtsam::zeros(3,3);
  for(size_t m = 0; m < 3; ++m) {
    for(size_t n = 0; n < 3; ++n) {
      match.cov(m,n) = gsl_matrix_get(output.cov_x_m, m, n);
    }
  }
  ROS_INFO_STREAM("r1");

  // Release additional allocated memory
  gsl_matrix_free(output.cov_x_m);
  gsl_matrix_free(output.dx_dy1_m);
  gsl_matrix_free(output.dx_dy2_m);
  ROS_INFO_STREAM("r1");

  // Add some error detection
  double initial_guess_error = initial_pose.localCoordinates(match.relative_pose).norm();
  if(initial_guess_error > initial_guess_error_threshold) throw std::runtime_error("Scanmatch deviation from initial guess is too large.");
  if(match.cov.trace() > covariance_trace_threshold) throw std::runtime_error("Scanmatch covariance is too large.");

  return match;
}

/* ************************************************************************* */
/*RelativePoseEstimates computeLaserScanMatches(
    const std::vector<sensor_msgs::LaserScan>& scans,
    struct sm_params& csm_params,
    const std::vector<gtsam::Pose2>& initial_guesses,
    const gtsam::Pose3& base_T_laser,
    double laserscan_sigma,
    double covariance_trace_threshold,
    double initial_guess_error_threshold,
    const std::string& csm_path)
{
  RelativePoseEstimates matches;

  // Check that the initial guesses have the same number of elements as scans (or are empty)
  if(!initial_guesses.empty() && (initial_guesses.size() != scans.size())) {
    throw(std::runtime_error("Initial guesses must have the same number of elements as scans."));
  }

  // Loop over all scans, creating scan pairs
  gtsam::Pose2 initial_guess = gtsam::Pose2::identity();
  std::string csm_filename = "";
  for(size_t i = 1; i < scans.size(); ++i) {
    // Create a custom log filename, if requested
    if(!csm_path.empty()) {
      std::ostringstream filename;
      filename << csm_path << "/match_" << std::fixed << std::setprecision(3) << scans.at(i-1).header.stamp.toSec() << "_" << std::fixed << std::setprecision(3) << scans.at(i).header.stamp.toSec() << ".json";
      csm_filename = filename.str();
    }

    try {
      // Compute the relative pose guess
      if(!initial_guesses.empty()) {
        initial_guess = initial_guesses.at(i-1).between(initial_guesses.at(i));
      }
      // Compute a single scan match
      RelativePoseEstimate match = computeLaserScanMatch(scans.at(i-1), scans.at(i), csm_params, initial_guess, base_T_laser, laserscan_sigma, covariance_trace_threshold, initial_guess_error_threshold, csm_filename);
      // Add the match
      matches.push_back(match);
    } catch(const std::exception& e) {
      ROS_WARN_STREAM("Error computing scan match from " << scans.at(i-1).header.stamp << " to " << scans.at(i).header.stamp << ".");
    }
  }

  return matches;
}
*/
/* ************************************************************************* */
/*RelativePoseEstimates computeLaserScanMatches(
    const std::vector<sensor_msgs::LaserScan>& scans1,
    const std::vector<sensor_msgs::LaserScan>& scans2,
    struct sm_params& csm_params,
    const std::vector<gtsam::Pose2>& initial_guesses,
    const gtsam::Pose3& base_T_laser,
    double laserscan_sigma,
    double covariance_trace_threshold,
    double initial_guess_error_threshold,
    const std::string& csm_path)
{
  RelativePoseEstimates matches;

  // Check that the two scan vectors have the same number of elements
  if(scans1.size() != scans2.size()) {
    throw(std::runtime_error("scans1 container and scans2 container must have the same number of elements."));
  }

  // Check that the initial guesses have the same number of elements as scans (or are empty)
  if(!initial_guesses.empty() && (initial_guesses.size() != scans1.size())) {
    throw(std::runtime_error("initial guesses must have the same number of elements as scans1 and scans2."));
  }

  // Loop over all scans
  gtsam::Pose2 initial_guess = gtsam::Pose2::identity();
  std::string csm_filename = "";
  for(size_t i = 0; i < scans1.size(); ++i) {
    // Create a custom log filename, if requested
    if(!csm_path.empty()) {
      std::ostringstream filename;
      filename << csm_path << "/match_" << std::fixed << std::setprecision(3) << scans1.at(i).header.stamp.toSec() << "_" << std::fixed << std::setprecision(3) << scans2.at(i).header.stamp.toSec() << ".json";
      csm_filename = filename.str();
    }

    try {
      // Compute the relative pose guess
      if(!initial_guesses.empty()) {
        initial_guess = initial_guesses.at(i);
      }
      // Compute a single scan match
      RelativePoseEstimate match = computeLaserScanMatch(scans1.at(i), scans2.at(i), csm_params, initial_guess, base_T_laser, laserscan_sigma, covariance_trace_threshold, initial_guess_error_threshold, csm_filename);
      // Add the match
      matches.push_back(match);
    } catch(const std::exception& e) {
      ROS_WARN_STREAM("Error computing scan match from " << scans1.at(i).header.stamp << " to " << scans2.at(i).header.stamp << ".");
    }
  }

  return matches;
}
*/
/* ************************************************************************* */
/*RelativePoseEstimates computeLaserScanMatches(
    const AugmentedLaserScans& augmented_scans,
    const TimestampPairs& pairs,
    struct sm_params& csm_params,
    const gtsam::Pose3& base_T_laser,
    double laserscan_sigma,
    double covariance_trace_threshold,
    double initial_guess_error_threshold,
    const std::string& csm_path,
    bool use_map_frame)
{
  RelativePoseEstimates matches;
  matches.reserve(pairs.size());

  Timer timer;
  timer.start();

  if(augmented_scans.empty()) {
    throw std::runtime_error("No augmented scans available to compute matches over.");
  }

  if(pairs.empty()) {
    throw std::runtime_error("No timestamp pairs available to compute matches over.");
  }

  // Loop over each provided laserscan pair
//#pragma omp parallel for
  for(size_t i = 0; i < pairs.size(); ++i) {

    // Create access to the specified timestamp pair
    const ros::Time& timestamp1 = pairs.at(i).first;
    const ros::Time& timestamp2 = pairs.at(i).second;

    // Look up the corresponding laserscan data
    AugmentedLaserScans::const_iterator augmented_scan1_iter = augmented_scans.find(timestamp1);
    AugmentedLaserScans::const_iterator augmented_scan2_iter = augmented_scans.find(timestamp2);

    if(augmented_scan1_iter == augmented_scans.end() || augmented_scan2_iter == augmented_scans.end()) {
      ROS_WARN_STREAM("Could not find scan data for the provided timestamp pair: timestamp1: " << timestamp1 << ", timestamp2: " << timestamp2);
      continue;
    }

    // Access to the augmented scan data
    const AugmentedLaserScan& augmented_scan1 = augmented_scan1_iter->second;
    const AugmentedLaserScan& augmented_scan2 = augmented_scan2_iter->second;

    // Create the initial relative pose guess
    gtsam::Pose2 first_guess;
    if(use_map_frame) {
      first_guess = augmented_scan1.map_pose.between(augmented_scan2.map_pose);
    } else {
      first_guess = augmented_scan1.odom_pose.between(augmented_scan2.odom_pose);
    }

    // Create the CSM log filename
    std::string csm_filename = "";
    if(!csm_path.empty()) {
      std::ostringstream filename;
      filename << csm_path << "/match_" << std::fixed << std::setprecision(3) << timestamp1.toSec() << "_" << std::fixed << std::setprecision(3) << timestamp2.toSec() << ".json";
      csm_filename = filename.str();
    }

    // Use CSM to calculate the relative pose and covariance between scan1 and scan2
    try {
      RelativePoseEstimate match = computeLaserScanMatch(augmented_scan1.scan, augmented_scan2.scan, csm_params, first_guess, base_T_laser, laserscan_sigma, covariance_trace_threshold, initial_guess_error_threshold, csm_filename);
      matches.push_back(match);
    } catch(const std::exception& e) {
      ROS_WARN_STREAM("Error computing scan match from " << timestamp1 << " to " << timestamp2 << ". Error: " << e.what());
    }
  }

  // Add the results to the output
  timer.stop();
  ROS_DEBUG_STREAM("Computed " << matches.size() << " laserscan matches using CSM in " << timer.elapsed() << " seconds.");

  return matches;
}
*/
///* ************************************************************************* */
//size_t ComputeScanCorrespondences(const gtsam::Pose2 relative_pose, const sensor_msgs::LaserScan& scan1, const sensor_msgs::LaserScan& scan2, double correspondence_distance) {
//  // Create the output value
//  size_t correspondence_count = 0;
//
//  // Create an R-Tree for efficient 2-D geometry lookups
//  typedef boost::geometry::model::d2::point_xy<double, boost::geometry::cs::cartesian> PointType;
//  typedef std::pair<PointType, size_t> RTreeValueType;
//  boost::geometry::index::rtree<RTreeValueType, boost::geometry::index::linear<16> > rtree;
//
//  // Insert the laser points from scan1
//  for(size_t i = 0; i < scan1.ranges.size(); ++i) {
//    double angle = scan1.angle_min + i*scan1.angle_increment;
//    double x = scan1.ranges.at(i)*cos(angle);
//    double y = scan1.ranges.at(i)*sin(angle);
//    rtree.insert(std::make_pair(PointType(x,y), i));
//  }
//
//  // Loop over the laser points from scan2, checking if a point from scan1 is close
//  for(size_t i = 0; i < scan2.ranges.size(); ++i) {
//    // Compute the transformed laserscan point
//    double angle = scan2.angle_min + i*scan2.angle_increment + relative_pose.theta();
//    double x = scan2.ranges.at(i)*cos(angle) + relative_pose.x();
//    double y = scan2.ranges.at(i)*sin(angle) + relative_pose.y();
//    PointType query_point(x,y);
//
//    // Query the R-Tree for the nearest point from scan1
//    std::vector<RTreeValueType> returned_values;
//    rtree.query(boost::geometry::index::nearest(query_point, 1), std::back_inserter(returned_values));
//
//    // Check the distance
//    if(!returned_values.empty() && boost::geometry::distance(query_point, returned_values[0].first) < correspondence_distance) {
//      ++correspondence_count;
//    }
//  }
//
//  return correspondence_count;
//}

///* ************************************************************************* */
//size_t ComputeScanCorrespondences(const gtsam::Pose2 relative_pose, const sensor_msgs::LaserScan& scan1, const sensor_msgs::LaserScan& scan2, double correspondence_distance) {
//  // Create the output value
//  size_t correspondence_count = 0;
//
//  // Create an occupancy grid
//  double delta = 2*correspondence_distance;
//  size_t grid_size = 2*std::floor(scan1.range_max/delta) + 1;
//  size_t center = std::floor(scan1.range_max/delta);
//  unsigned char grid[grid_size][grid_size];
//  memset(grid, 0, sizeof(grid[0][0])*grid_size*grid_size);
//
//  // Insert the laser points from scan1 into the occupancy grid
//  for(size_t i = 0; i < scan1.ranges.size(); ++i) {
//    double angle = scan1.angle_min + i*scan1.angle_increment;
//    double x = scan1.ranges.at(i)*cos(angle);
//    double y = scan1.ranges.at(i)*sin(angle);
//    size_t u = (x/delta + 0.5) + center;
//    size_t v = (y/delta + 0.5) + center;
//    grid[u][v] = 1;
//  }
//
//  // Loop over the laser points from scan2, checking if the grid cell is occupied
//  for(size_t i = 0; i < scan2.ranges.size(); ++i) {
//    // Compute the transformed laserscan point
//    double angle = scan2.angle_min + i*scan2.angle_increment + relative_pose.theta();
//    double x = scan2.ranges.at(i)*cos(angle) + relative_pose.x();
//    double y = scan2.ranges.at(i)*sin(angle) + relative_pose.y();
//    size_t u = (x/delta + 0.5) + center;
//    size_t v = (y/delta + 0.5) + center;
//    if(grid[u][v]) {
//      ++correspondence_count;
//    }
//  }
//
//  return correspondence_count;
//}

///* ************************************************************************* */
//RelativePoseEstimates FilterLowOverlapMatches(const AugmentedScans& augmented_scans, const RelativePoseEstimates& relative_pose_estimates, double correspondence_distance, size_t minimum_correspondences, const std::string& debug_log) {
//  RelativePoseEstimates valid_relative_pose_estimates;
//  valid_relative_pose_estimates.reserve(relative_pose_estimates.size());
//
//  Timer timer;
//  timer.start();
//
//  if(augmented_scans.empty()) {
//    throw std::runtime_error("No augmented scans available to compute overlaps.");
//  }
//
//  if(relative_pose_estimates.empty()) {
//    throw std::runtime_error("No relative poses available to compute overlaps.");
//  }
//
//  std::ofstream logfile;
//  if(!debug_log.empty()) {
//    // create an output file
//    boost::filesystem::path file_path(debug_log);
//    boost::filesystem::create_directories(file_path.parent_path());
//    logfile.open(debug_log.c_str());
//  }
//
//  // Loop over each pair, checking that the provided relative pose does result in a high amount of overlap between scans
//  for(size_t i = 0; i < relative_pose_estimates.size(); ++i) {
//
//    // Check if a relative pose was computed successfully
//    if(relative_pose_estimates.at(i).timestamp1 == ros::Time() && relative_pose_estimates.at(i).timestamp2 == ros::Time()) {
//      continue;
//    }
//
//    // Look up the corresponding laserscan data
//    AugmentedScans::const_iterator augmented_scans1_iter = augmented_scans.find(relative_pose_estimates.at(i).timestamp1);
//    AugmentedScans::const_iterator augmented_scans2_iter = augmented_scans.find(relative_pose_estimates.at(i).timestamp2);
//
//    if(augmented_scans1_iter == augmented_scans.end() || augmented_scans2_iter == augmented_scans.end()) {
//      std::cout << "Could not find augmented scan data for the provided relative pose: timestamp1: " << relative_pose_estimates.at(i).timestamp1 << ", timestamp2: " << relative_pose_estimates.at(i).timestamp2 << std::endl;
//      continue;
//    }
//
//    // Access to the scan messages
//    const sensor_msgs::LaserScan& scan1 = augmented_scans1_iter->second.scan;
//    const sensor_msgs::LaserScan& scan2 = augmented_scans2_iter->second.scan;
//
//    // Compute the number of scan-scan correspondences
//    size_t correspondence_count = ComputeScanCorrespondences(relative_pose_estimates.at(i).relative_pose, scan1, scan2, correspondence_distance);
//
//    // Check the number of valid correspondences. If there are not enough points, delete the relative pose
//    if(correspondence_count > minimum_correspondences) {
//      valid_relative_pose_estimates.push_back(relative_pose_estimates.at(i));
//    }
//
//    if(logfile.is_open()) {
//      logfile << relative_pose_estimates.at(i).timestamp1 << " " << relative_pose_estimates.at(i).timestamp2 << " " << correspondence_count << std::endl;
//    }
//
//    // Print a progress message
//    printProgressBar("  Validating Laserscan Matches: ", 100 * (double)(i) / relative_pose_estimates.size());
//  }
//
//  // Close the debug log file
//  if(logfile.is_open()) {
//    logfile.close();
//  }
//
//  // Add the results to the output
//  timer.stop();
//  std::cout << ", Valid Laserscan Matches: " << valid_relative_pose_estimates.size() << ", Time: " << timer.elapsed() << std::endl;
//
//  return valid_relative_pose_estimates;
//}

/* ************************************************************************* */
void streamCsmScan(std::ofstream& stream, struct laser_data* scan) {

  stream << "{ ";
  stream << "\"nrays\": " << scan->nrays << ", ";
  stream << "\"min_theta\": " << scan->min_theta << ", ";
  stream << "\"max_theta\": " << scan->max_theta << ", ";

  stream << "\"odometry\": [ ";
  for(size_t i = 0; i < 3; ++i) {
    if(i > 0) stream << ", ";
    if(std::isnan(scan->odometry[i]))
      stream << "null";
    else
      stream << scan->odometry[i];
  }
  stream << " ], ";

  stream << "\"estimate\": [ ";
  for(size_t i = 0; i < 3; ++i) {
    if(i > 0) stream << ", ";
    if(std::isnan(scan->estimate[i]))
      stream << "null";
    else
      stream << scan->estimate[i];
  }
  stream << " ], ";

  stream << "\"theta\": [ ";
  for(size_t i = 0; i < scan->nrays; ++i) {
    if(i > 0) stream << ", ";
    if(std::isnan(scan->theta[i]))
      stream << "null";
    else
      stream << scan->theta[i];
  }
  stream << "], ";

  stream << "\"readings\": [ ";
  for(size_t i = 0; i < scan->nrays; ++i) {
    if(i > 0) stream << ", ";
    if(std::isnan(scan->readings[i]))
      stream << "null";
    else
      stream << scan->readings[i];
  }
  stream << "], ";

  stream << "\"valid\": [ ";
  for(size_t i = 0; i < scan->nrays; ++i) {
    if(i > 0) stream << ", ";
    if(std::isnan(scan->readings[i]))
      stream << 0;
    else
      stream << scan->valid[i];
  }
  stream << "]";

  stream << " }" << std::endl;
}

/* ************************************************************************* */
void writeCsmLog(struct laser_data* reference, struct laser_data* scan, const std::string& filename) {

  // create an output file
  boost::filesystem::path file_path(filename);
  boost::filesystem::create_directories(file_path.parent_path());
  std::ofstream logfile(filename.c_str());

  // Write the two scans to the log
  streamCsmScan(logfile, reference);
  streamCsmScan(logfile, scan);

  // Close the logfile
  logfile.close();
}

/* ************************************************************************* */
} /// @namespace csm

} /// @namespace bnr_mapping


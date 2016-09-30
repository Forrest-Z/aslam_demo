/**
+ * mapping_common.h
 */

#ifndef MAPPING_COMMON_H
#define MAPPING_COMMON_H

#include <aslam_demo/mapping/mapping_common.h>
#include <tf2_ros/buffer.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <boost/array.hpp>
#include <map>
#include <set>
#include <vector>
#include <utility>

namespace mapping {

typedef struct {
  ros::Time timestamp1;
  ros::Time timestamp2;
  gtsam::Pose2 relative_pose;
  gtsam::Matrix cov;
} RelativePoseEstimate;

typedef struct {
  gtsam::Pose2 odom_pose;
  gtsam::Pose2 map_pose;
  sensor_msgs::LaserScan scan;
  sensor_msgs::PointCloud2 cloud;
} AugmentedLaserScan;

typedef std::set<ros::Time> Timestamps;
typedef std::pair<ros::Time, ros::Time> TimeRange;
typedef std::pair<ros::Time, ros::Time> TimestampPair;
typedef std::vector<TimestampPair> TimestampPairs;
typedef std::multimap<ros::Time, geometry_msgs::TransformStamped> TfTransforms;
typedef std::map<ros::Time, nav_msgs::Odometry> Odometry;
typedef std::map<ros::Time, sensor_msgs::LaserScan> LaserScans;
typedef std::map<ros::Time, geometry_msgs::PoseWithCovarianceStamped> PoseWithCovariances;
typedef std::map<ros::Time, AugmentedLaserScan> AugmentedLaserScans;
typedef std::map<ros::Time, sensor_msgs::PointCloud2> PointClouds;
typedef std::map<ros::Time, gtsam::Pose2> Poses;
typedef std::map<ros::Time, sensor_msgs::LaserScan> LaserScans;
typedef std::vector<RelativePoseEstimate> RelativePoseEstimates;

typedef struct {
	gtsam::NonlinearFactorGraph factor_graph;
	gtsam::KeySet keys;
} graphWithKeys;
/**
 * Print a progress bar on the screen that gradually fills in
 * @param title
 * @param percent
 */
void printProgressBar(const std::string& title, int percent);

/**
 * Open all of the bagfiles indicated by filenames
 * @param filenames A container of bag filenames to open
 * @param bags [output] A container of open bag files
 */
void openBagfiles(const std::vector<std::string>& filenames, std::vector<rosbag::Bag>& bags);

/**
 * Close all bagfiles in the provided container
 * @param A container of open bag files
 */
void closeBagfiles(std::vector<rosbag::Bag>& bags);

/**
 * Combine multiple bagfiles into a single view, filtering out just the speciified topics
 * @param bags
 * @param topics
 * @param time_range
 * @return
 */
boost::shared_ptr<rosbag::View> queryBagfiles(const std::vector<rosbag::Bag>& bags, const std::vector<std::string>& topics = std::vector<std::string>(),
    const TimeRange& time_range = TimeRange(ros::TIME_MIN, ros::TIME_MAX));

/**
 * Read all static transforms from the provided bagfiles and insert them into a tf2_ros::Buffer object.
 * @param bags
 * @param tf_buffer
 */
void loadStaticTf(const std::vector<rosbag::Bag>& bags, tf2_ros::Buffer& tf_buffer);

/**
 * @brief Populate a completed_mission_parser::CompletedMission object based on the series of possible inputs
 *
 * There are several methods for populating the CompletedMission object; some consume far more resources than others.
 * This function attempts to populate the CompletedMission object using several different input sources, prioritized
 * by the size of the data required (aisles.txt, sparse bagfiles, shelf bagfiles). If none of the sources contain
 * valid mission information, this function will return an empty mission object.
 *
 * @param configuration A smoothing configuration object containing all of the settings used in constraint generation
 * @return The completed_mission_parser::CompletedMission object populated with the requested set of aisles
 */

/**
 * Extract the time range covered by the provided bag files
 * @param bags
 * @return
 */
TimeRange extractTimeRange(const std::vector<rosbag::Bag>& bags);

/**
 * Extract all tf messages from the bagfiles
 * @param bags
 * @param time_range
 * @return
 */
TfTransforms extractTfTransforms(const std::vector<rosbag::Bag>& bags, const TimeRange& time_range, const ros::Duration tf_buffer = ros::Duration(5.0));

/**
 * Extract all odometry messages from the bagfiles between the provided timestamps
 * @param bags
 * @param time_range
 * @param topic
 * @return
 */
Odometry extractOdometry(const std::vector<rosbag::Bag>& bags, const TimeRange& time_range, const std::string& topic = "/odom");

/**
 * Extract all laserscan messages from the bagfiles
 * @param bags
 * @param time_range
 * @param topic
 * @return
 */
LaserScans extractLaserScans(const std::vector<rosbag::Bag>& bags, const TimeRange& time_range, const std::string& topic = "/scan");

/**
 * Extract all pointcloud messages from the bagfiles
 * @param bags
 * @param time_range
 * @param topic
 * @return
 */
PointClouds extractPointclouds(const std::vector<rosbag::Bag>& bags, const TimeRange& time_range, const std::string& topic = "/cloud");

/**
 * Extract all pose messages from the bagfiles
 * @param bags
 * @param time_range
 * @param topic
 * @param default_covariance
 * @return
 */
PoseWithCovariances extractPoses(const std::vector<rosbag::Bag>& bags, const TimeRange& time_range, const std::string& topic, const boost::array<double, 36>& default_covariance);

/**
 * Extract the unique timestamps from one of the bnr_mapping datatypes
 * @param time_keyed_container
 * @return
 */
template<class T>
Timestamps extractTimestamps(const std::map<ros::Time, T>& time_keyed_container);

/**
 * Extract the unique timestamps from one of the bnr_mapping datatypes
 * @param time_keyed_container
 * @return
 */
template<class T>
Timestamps extractTimestamps(const std::multimap<ros::Time, T>& time_keyed_container);

/**
 * Find the closest entry to the provided timestamp within a Time-Value map.
 * An optional time tolerance is supported; only values within +/- tolerance
 * are considered.
 * @param map
 * @param query_time
 * @param tolerance
 * @return
 */
Timestamps::const_iterator findClosest(const Timestamps& timestamps, const ros::Time& query_time, boost::optional<double> tolerance = boost::none);

/**
 * Find the closest entry to the provided timestamp within a Time-Value map.
 * An optional time tolerance is supported; only values within +/- tolerance
 * are considered.
 * @param map
 * @param query_time
 * @param tolerance
 * @return
 */
template<class T>
typename std::map<ros::Time, T>::const_iterator findClosest(const std::map<ros::Time, T>& map, const ros::Time& query_time, boost::optional<double> tolerance = boost::none);

/**
 * Find the closest entry to the provided timestamp within a Time-Value map.
 * An optional time tolerance is supported; only values within +/- tolerance
 * are considered.
 * @param map
 * @param query_time
 * @param tolerance
 * @return
 */
template<class T>
typename std::multimap<ros::Time, T>::const_iterator findClosest(const std::multimap<ros::Time, T>& map, const ros::Time& query_time, boost::optional<double> tolerance = boost::none);

/**
 * Extract a 2D robot pose for each timestamp in the provided container
 * @param timestamps
 * @param transforms
 * @param base_frame
 * @param global_frame
 * @return
 */
Poses computePoses(const Timestamps& timestamps, const TfTransforms& transforms, const std::string& global_frame = "map", const std::string& base_frame = "base_link");

///**
// * Serialize the pose values to a log file for easy MATLAB parsing.
// * Format: Timestamp Variable(e.g. x y theta)
// * @param filename The filename of the CSV log to create
// * @param poses The set of poses to serialize
// */
//void WritePoses(const std::string& filename, const Poses& poses);

/**
 * Serialize the extracted scans to a PLY-format log file
 * @param scans The set of scans to serialize
 * @param filename The PLY filename to generate
 * @param use_binary_format Flag indicating the PLY file should be binary
 */
void writePointcloudPLY(const PointClouds& clouds, const std::string& filename, bool use_binary_format = false);

/**
 * Serialize the extracted scans to a MAtlab-friendly log file
 * @param scans The set of scans to serialize
 */
void writePointcloudCSV(const PointClouds& clouds, const std::string& filename);

/* ************************************************************************* */
template<class T>
Timestamps extractTimestamps(const std::map<ros::Time, T>& time_keyed_container) {
  Timestamps timestamps;

  // Extract the unique timestamps from the provided container
  for(typename std::map<ros::Time, T>::const_iterator iter = time_keyed_container.begin(); iter != time_keyed_container.end(); ++iter) {
    timestamps.insert(iter->first);
  }

  return timestamps;
}

/* ************************************************************************* */
template<class T>
Timestamps extractTimestamps(const std::multimap<ros::Time, T>& time_keyed_container) {
  Timestamps timestamps;

  // Extract the unique timestamps from the provided container
  for(typename std::multimap<ros::Time, T>::const_iterator iter = time_keyed_container.begin(); iter != time_keyed_container.end(); ++iter) {
    timestamps.insert(iter->first);
  }

  return timestamps;
}

/* ************************************************************************* */
template<class T>
typename std::map<ros::Time, T>::const_iterator findClosest(const std::map<ros::Time, T>& map, const ros::Time& query_time, boost::optional<double> tolerance) {
  // Set the initial and final position of the search
  typename std::map<ros::Time, T>::const_iterator map_begin;
  typename std::map<ros::Time, T>::const_iterator map_end;
  if(tolerance) {
    map_begin = map.lower_bound(query_time - ros::Duration(tolerance.get()));
    map_end   = map.upper_bound(query_time + ros::Duration(tolerance.get()));
  } else {
    map_begin = map.begin();
    map_end   = map.end();
  }

  // Perform the search
  double min_delta = std::numeric_limits<double>::max();
  typename std::map<ros::Time, T>::const_iterator min_iter = map.end();
  for(typename std::map<ros::Time, T>::const_iterator iter = map_begin; iter != map_end; ++iter) {
    // Compute the time delta to the query timestamp
    double delta = std::fabs( (iter->first - query_time).toSec() );
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
  // If the container is empty, or no entries exist within the provided time tolerance, this iterator will point to map.end()
  return min_iter;
}

/* ************************************************************************* */
template<class T>
typename std::multimap<ros::Time, T>::const_iterator findClosest(const std::multimap<ros::Time, T>& map, const ros::Time& query_time, boost::optional<double> tolerance) {
  // Set the initial and final position of the search
  typename std::multimap<ros::Time, T>::const_iterator map_begin;
  typename std::multimap<ros::Time, T>::const_iterator map_end;
  if(tolerance) {
    map_begin = map.lower_bound(query_time - ros::Duration(tolerance.get()));
    map_end   = map.upper_bound(query_time + ros::Duration(tolerance.get()));
  } else {
    map_begin = map.begin();
    map_end   = map.end();
  }

  // Perform the search
  double min_delta = std::numeric_limits<double>::max();
  typename std::multimap<ros::Time, T>::const_iterator min_iter = map.end();
  for(typename std::multimap<ros::Time, T>::const_iterator iter = map_begin; iter != map_end; ++iter) {
    // Compute the time delta to the query timestamp
    double delta = std::fabs( (iter->first - query_time).toSec() );
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
  // If the container is empty, or no entries exist within the provided time tolerance, this iterator will point to map.end()
  return min_iter;
}

/* ************************************************************************* */
} /// @namespace bnr_mapping

#endif // MAPPING_COMMON_H


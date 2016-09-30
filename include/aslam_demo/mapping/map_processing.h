/**
 * map_processing.h
 */

#ifndef MAP_PROCESSING_H
#define MAP_PROCESSING_H

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
#include <aslam_demo/mapping/probability_map.h>
#include <aslam_demo/mapping/mapping_common.h>

#include <nav_msgs/OccupancyGrid.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose3.h>
#include <string>

namespace mapping {

namespace map {

/**
 * Create an empty map large enough for all of the poses plus the specified buffer.
 * Depending on the included poses, the map may not actually span the origin.
 * @param values The set of optimized poses used to build the map
 * @return An empty map large enough to hold all the provided poses
 */
ProbabilityMap createEmptyMap(const gtsam::Values& values, double map_cell_size, double map_size_buffer);

/**
 * Update the map with the provided laser scans at the optimized poses.
 * @param map
 * @param values
 * @param scans
 */
void buildMap(ProbabilityMap& map, const gtsam::Values& values, const LaserScans& scans, const gtsam::Pose3& base_T_laser, double scan_sigma, double time_tolerance, const std::string& debug_path);



/* ************************************************************************* */
/* ************************************************************************* */
/* ************************************************************************* */
// nav_msgs::OccupancyGrid processing functions

///**
// *
// * @param map_yaml_file
// * @param map
// */
//void readMap(const std::string& map_yaml_file, nav_msgs::OccupancyGrid& map);

/**
 * @brief Create an empty nav_msgs::OccupancyGrid map with the same settings and size as the provided map description YAML file
 *
 * @param map_yaml_file A ROS map server YAML file describing an existing map (scale, origin, etc)
 * @param map A reference to a nav_msgs::OccupancyGrid object. This object will be modified. Any existing data will be
 *            cleared, and the data will be resized to match the provided map file
 */
void createMap(const std::string& map_yaml_file, nav_msgs::OccupancyGrid& map);

/**
 * @brief Save a nav_msgs::OccupancyGrid map as an image file.
 *
 * This function uses the ImageMagick library to write the output file. Many/most image formats are supported. The format is automatically
 * deduced from the file using logic internal to the ImageMagick library.
 * The output will be grayscale. Any value in the nav_msgs::OccupancyGrid less than the free_threshold percentage will be written as white
 * pixels. Any value in the nav_msgs::OccupancyGrid greater than the occupied_threshold percentage will be written as black pixels. All other
 * pixel values will be linearly scaled between the free_threshold and the occupied_threshold.
 * @param image_file The full path and filename of the output image, including image extension. ImageMagick uses the extension to determine the requested image format.
 * @param map The nav_msgs::OccupancyGrid to write to an image file
 * @param free_threshold Any value in the nav_msgs::OccupancyGrid less than this percentage will be written as white pixels
 * @param occupied_threshold Any value in the nav_msgs::OccupancyGrid greater than this percentage will be written as black pixels
 */
void writeMap(const std::string& image_file, const nav_msgs::OccupancyGrid& map, double free_threshold = 0.1, double occupied_threshold = 0.65);

/**
 * @brief Resize an existing nav_msgs::OccupancyGrid.
 *
 * The origin of the nav_msgs::OccupancyGrid will be updated to (min_x, min_y). The size of the nav_msgs::OccupancyGrid data will be resized
 * to hold the new map extents, based on the current map resolution. The map can be resized larger or smaller. All data from the previous map
 * will copied into the correct pixel location of the new map, based on the world position of each map pixel.
 * @param map The nav_msgs::OccupancyGrid object to resize
 * @param min_x The minimum x-value to hold in the map, in world coordinates
 * @param max_x The maximum x-value to hold in the map, in world coordinates
 * @param min_y The minimum y-value to hold in the map, in world coordinates
 * @param max_y The maximum y-value to hold in the map, in world coordinates
 */
void resizeMap(nav_msgs::OccupancyGrid& map, double min_x, double max_x, double min_y, double max_y);

/**
 * @brief Reset all pixels of a nav_msgs::OccupancyGrid to zero
 *
 * @param map The nav_msgs::OccupancyGrid object to clear
 */
void clearMap(nav_msgs::OccupancyGrid& map);

/**
 * @brief Mark a specific world coordinate in a nav_msgs::OccupancyGrid as fully occupied
 *
 * @param map The nav_msgs::OccupancyGrid object to update
 * @param point The point, in world coordinates, to mark as occupied
 */
void markMap(nav_msgs::OccupancyGrid& map, const geometry_msgs::Point& point);

/**
 * @brief Mark a set of world coordinates in a nav_msgs::OccupancyGrid as fully occupied
 *
 * @param map The nav_msgs::OccupancyGrid object to update
 * @param cloud A sensor_msgs::Pointcloud2 object containing the occupied points in world coordinates
 */
void markMap(nav_msgs::OccupancyGrid& map, const sensor_msgs::PointCloud2& cloud);

/**
 * @brief Check if a given world coordinate point is inside the bounds of the nav_msgs::OccupancyGrid
 *
 * @param map The nav_msgs::OccupancyGrid object
 * @param x The x-value in world coordinates
 * @param y The y-value in world coordinates
 * @return True if the point is inside the map, False otherwise
 */
bool insideMap(const nav_msgs::OccupancyGrid& map, double x, double y);

} /// @namespace map

} /// @namespace mapping

#endif // MAP_PROCESSING_H


/**
 * map_processing.cpp
 */

#include <aslam_demo/mapping/map_processing.h>
#include <aslam_demo/mapping/sensor_models.h>
#include <aslam_demo/mapping/timer.h>
#include <aslam_demo/factors/key_generator.h>
#include <ros/ros.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>
#include <opencv2/highgui/highgui.hpp>
#include <yaml-cpp/yaml.h>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

namespace mapping {

namespace map {

/* ************************************************************************* */
ProbabilityMap createEmptyMap(const gtsam::Values& values, double map_cell_size, double map_size_buffer) {

  if(values.empty()) {
    throw std::runtime_error("No poses available for map initialization.");
  }

  // Create a key generator for timestamp <--> key conversions
  // For this purpose, the time tolerance doesn't matter
  factors::KeyGenerator key_generator(1.0);

  // Loop through all of the values, extracting the bounding box
  double x_min = +std::numeric_limits<double>::max();
  double x_max = -std::numeric_limits<double>::max();
  double y_min = +std::numeric_limits<double>::max();
  double y_max = -std::numeric_limits<double>::max();
  BOOST_FOREACH(const gtsam::Values::ConstKeyValuePair& key_value, values) {
    // Only check Pose2 values
    if( key_generator.extractKeyType(key_value.key) == factors::key_type::Pose2) {
      // Extract the pose
      gtsam::Pose2 pose = static_cast<const gtsam::Pose2&>(key_value.value);
      // Update the bounding box
      if(pose.x() < x_min) x_min = pose.x();
      if(pose.x() > x_max) x_max = pose.x();
      if(pose.y() < y_min) y_min = pose.y();
      if(pose.y() > y_max) y_max = pose.y();
    }
  }

  // Increase the bounding box by the provided buffer
  x_min -= map_size_buffer;
  x_max += map_size_buffer;
  y_min -= map_size_buffer;
  y_max += map_size_buffer;

  // Calculate width, height, and origin
  size_t cols = std::ceil((x_max - x_min) / map_cell_size);
  size_t rows = std::ceil((y_max - y_min) / map_cell_size);
  gtsam::Point2 origin(x_min, y_min);

  // Create the map
  ProbabilityMap map(rows, cols, map_cell_size, origin);

  return map;
}

/* ************************************************************************* */
void buildMap(ProbabilityMap& map, const gtsam::Values& values, const LaserScans& scans, const gtsam::Pose3& base_T_laser, double scan_sigma, double time_tolerance, const std::string& debug_path) {

  Timer timer;
  timer.start();

  // Create map directory
  boost::filesystem::path dir(debug_path);
  boost::filesystem::create_directory(dir);

  // Create a laser model for map updates
  sensor_models::LaserScanModel laser_model(scan_sigma, false);

  // Create a key generator for timestamp <--> key conversions
  factors::KeyGenerator key_generator(time_tolerance);

  // Loop through the optimized poses
  size_t counter = 0;
  BOOST_FOREACH(const gtsam::Values::ConstKeyValuePair& key_value, values) {
    // Extract the key type and timestamp
    factors::key_type::Enum key_type = key_generator.extractKeyType(key_value.key);
    ros::Time timestamp = key_generator.extractTimestamp(key_value.key);
    ROS_INFO_STREAM("Am here"<<timestamp);

    // Only use Pose2 values
    if( key_type == factors::key_type::Pose2) {

      // Find the laserscan closest to the pose timestamp
      LaserScans::const_iterator scans_begin = scans.lower_bound(timestamp - ros::Duration(time_tolerance));
      LaserScans::const_iterator scans_end   = scans.upper_bound(timestamp + ros::Duration(time_tolerance));
      ROS_INFO_STREAM("Am here 1"<<scans.lower_bound(timestamp - ros::Duration(time_tolerance))->first<<"\t"<<scans.upper_bound(timestamp + ros::Duration(time_tolerance))->first);
      ROS_INFO_STREAM("WW"<<timestamp - ros::Duration(time_tolerance)<<"\t"<<timestamp + ros::Duration(time_tolerance));
      ROS_INFO_STREAM("TT"<<scans.begin()->first<<"\t"<<std::next(scans.begin(),1)->first<<"\t"<<std::next(scans.begin(),2)->first<<"\t"<<std::next(scans.begin(),3)->first<<"\t"<<std::next(scans.begin(),4)->first);
      const sensor_msgs::LaserScan* scan = 0;
      double min_delta = std::numeric_limits<double>::max();
      for(LaserScans::const_iterator scans_iter = scans_begin; scans_iter != scans_end; ++scans_iter) {
        ROS_INFO_STREAM("Am here 2");

        double delta = std::fabs( (scans_iter->first - timestamp).toSec() );
        if(delta < min_delta) {
          min_delta = delta;
          scan = &(scans_iter->second);
        }
      }
      ROS_INFO_STREAM("Am here 3"<<*scan);
      // If a scan was found, add it to the map
      if(scan) {
          ROS_INFO_STREAM("Am here 4");
          key_value.value.print();

        // Look up the optimized pose
        gtsam::Pose2 world_T_base = static_cast<const gtsam::Pose2&>(key_value.value);

        world_T_base.print("World_T_base: ");
        // Update the map use the laser scan model
        laser_model.updateMap(map, *scan, world_T_base, base_T_laser);
      }
    }

    // Periodically save an intermediate map
    size_t delta = std::floor(values.size() / 10);
    if(counter % delta == 0) {
      map.occupancyGrid(debug_path + "/map-" + boost::lexical_cast<std::string>(timestamp.toSec()));
    }

    // Print a progress message
   // printProgressBar("  Building Map: ", 100 * (double)(counter++) / values.size());
  }

  // Add the results to the output
  timer.stop();
  std::cout << ", Time: " << timer.elapsed() << std::endl;
}



/* ************************************************************************* */
/* ************************************************************************* */
/* ************************************************************************* */
void createMap(const std::string& map_yaml_file, nav_msgs::OccupancyGrid& map) {
  // Set default values for all map info parameters
  map.header.frame_id = "map";
  map.info.width = 0;
  map.info.height = 0;
  map.info.resolution = 0.02;
  map.info.origin.position.x = 0.0;
  map.info.origin.position.y = 0.0;
  map.info.origin.position.z = 0.0;
  map.info.origin.orientation.w = 1.0;
  map.info.origin.orientation.x = 0.0;
  map.info.origin.orientation.y = 0.0;
  map.info.origin.orientation.z = 0.0;

  // Load the provided map yaml file
  YAML::Node map_config = YAML::LoadFile(map_yaml_file);

  // Update the map info based on the yaml file information
  if(map_config["resolution"]) map.info.resolution = map_config["resolution"].as<double>();
  if(map_config["origin"]) {
    map.info.origin.position.x = map_config["origin"][0].as<double>();
    map.info.origin.position.y = map_config["origin"][1].as<double>();
    // Ignore the origin yaw. Nothing supports rotated maps.
  }

  // Find the map image file referenced in the yaml file
  boost::filesystem::path image_path = boost::filesystem::path(map_yaml_file).parent_path() / boost::filesystem::path(map_config["image"].as<std::string>());

  // Load the existing map to determine the map size
  cv::Mat image = cv::imread(image_path.string());

  // Update the map size based on the image
  map.info.width = image.cols;
  map.info.height = image.rows;
  map.data.resize(map.info.width*map.info.height, 0.0);
}

/* ************************************************************************* */
void writeMap(const std::string& filename, const nav_msgs::OccupancyGrid& map, double free_threshold, double occupied_threshold) {
  // Create an empty image of the correct size
  cv::Mat image(map.info.height, map.info.width, CV_8UC1, cv::Scalar(255));

  // Populate the image with data from the OccupancyGrid
  for(unsigned int y = 0; y < map.info.height; y++) {
    uchar* pixel_row = image.ptr<uchar>(y); // point to first pixel in row
    for(unsigned int x = 0; x < map.info.width; x++) {
      // Compute the OccupancyGrid. For some reason the image is reflected vertically
      unsigned int i = x + (map.info.height - y - 1) * map.info.width;
      // Compute the pixel color
      uchar color;
      if(map.data[i] > occupied_threshold*100) {
        color = 0;
      } else if(map.data[i] < free_threshold*100) {
        color = 255;
      } else {
        color = (uchar)(255*(1.0 - ((double)map.data[i]/100.0 - free_threshold)/(occupied_threshold - free_threshold)));
      }
      // Update the image
      pixel_row[x] = color;
    }
  }

  // Save the image
  cv::imwrite(filename, image);
}

/* ************************************************************************* */
void resizeMap(nav_msgs::OccupancyGrid& map, double min_x, double max_x, double min_y, double max_y) {
  // Compute the size of the new map in pixels
  size_t new_width = ((max_x - min_x) / map.info.resolution) + 0.5;
  size_t new_height = ((max_y - min_y) / map.info.resolution) + 0.5;

  // Create a new data container of the correct size
  std::vector<int8_t> new_data;
  new_data.resize(new_width*new_height, 0);

  // Populate the new map data with the pixel values from the old map data
  for(size_t x = 0; x < map.info.width; ++x) {
    for(size_t y = 0; y < map.info.height; ++y) {
      double world_x = map.info.origin.position.x + x*map.info.resolution;
      double world_y = map.info.origin.position.y + y*map.info.resolution;
      // Check that the old point is within the bounds of the new map
      if( (world_x < min_x) || (world_x > max_x) || (world_y < min_y) || (world_y > max_y) ) {
        continue;
      }
      // Compute the pixel coordinates in the old and new map
      size_t old_pixel_index = x + (map.info.width)*y;
      size_t new_pixel_index = ((int)((world_x - min_x)/map.info.resolution)) + (new_width)*((int)((world_y - min_y)/map.info.resolution));
      new_data[new_pixel_index] = map.data[old_pixel_index];
    }
  }

  // Update the map with the new information
  map.info.origin.position.x = min_x;
  map.info.origin.position.y = min_y;
  map.info.origin.position.z = 0.0;
  map.info.origin.orientation.w = 1.0;
  map.info.origin.orientation.x = 0.0;
  map.info.origin.orientation.y = 0.0;
  map.info.origin.orientation.z = 0.0;
  map.info.width = new_width;
  map.info.height = new_height;
  map.data.swap(new_data);
}

/* ************************************************************************* */
void clearMap(nav_msgs::OccupancyGrid& map) {
  for(unsigned int y = 0; y < map.info.height; y++) {
    for(unsigned int x = 0; x < map.info.width; x++) {
      unsigned int i = x + map.info.height*map.info.width;
      map.data[i] = 0;
    }
  }
}

/* ************************************************************************* */
void markMap(nav_msgs::OccupancyGrid& map, const geometry_msgs::Point& point) {
  // Convert world point (x,y) into a map index
  if(!insideMap(map, point.x, point.y)) {
    return;
  }
  unsigned int mx = (int)((point.x - map.info.origin.position.x) / map.info.resolution);
  unsigned int my = (int)((point.y - map.info.origin.position.y) / map.info.resolution);
  unsigned int index = my*map.info.width + mx;
  // Mark the map as occupied
  map.data[index] = 100;
}

/* ************************************************************************* */
void markMap(nav_msgs::OccupancyGrid& map, const sensor_msgs::PointCloud2& cloud) {
  // Find the x and y channel offsets
  unsigned int x_offset = std::numeric_limits<unsigned int>::max();
  unsigned int y_offset = std::numeric_limits<unsigned int>::max();
  for(size_t i = 0; i < cloud.fields.size(); ++i) {
    if(cloud.fields.at(i).name == "x") x_offset = cloud.fields.at(i).offset;
    if(cloud.fields.at(i).name == "y") y_offset = cloud.fields.at(i).offset;
  }
  // Check that the required offset are valid
  if((x_offset == std::numeric_limits<unsigned int>::max()) || (y_offset == std::numeric_limits<unsigned int>::max())) {
    throw(std::runtime_error("Supplied pointcloud does not have X,Y fields."));
  }
  // Loop over the points in the pointcloud, marking each X,Y point in the map
  for(size_t row = 0; row < cloud.height; ++row) {
    for(size_t col = 0; col < cloud.width; ++col) {
      size_t point_offset = row*cloud.row_step + col*cloud.point_step;
      geometry_msgs::Point point;
      point.x = *(float*)(&cloud.data.at(point_offset + x_offset));
      point.y = *(float*)(&cloud.data.at(point_offset + y_offset));
      point.z = 0.0;
      markMap(map, point);
    }
  }
}

/* ************************************************************************* */
bool insideMap(const nav_msgs::OccupancyGrid& map, double x, double y) {
  return (x >= map.info.origin.position.x) && (x < map.info.origin.position.x + map.info.width*map.info.resolution)
      && (y >= map.info.origin.position.y) && (y < map.info.origin.position.y + map.info.height*map.info.resolution);
}

/* ************************************************************************* */
} /// @namespace map

} /// @namespace mapping


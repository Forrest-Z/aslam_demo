/**
 * sensor_models.h
 */

#ifndef SENSOR_MODELS_H
#define SENSOR_MODELS_H

#include <aslam_demo/mapping/probability_map.h>
#include <sensor_msgs/LaserScan.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>

namespace bnr_mapping {

namespace sensor_models {

/**
 * A class that encapsulates the Laser Scan sensor model used for map construction
 */
class LaserScanModel {
public:

  /**
   * Default constructor
   * @param range_sigma Standard deviation of the laser range measurement
   * @param clearing_probability Probability a cell is empty if a laser pass through it [0.0 1.0]
   * @param use_max_range
   */
  LaserScanModel(double range_sigma = 0.05, bool use_max_range = false);

  /**
   * Destructor
   */
  virtual ~LaserScanModel();

  /**
   * Update the map with a single laser return
   * @param map The map to update
   * @param sensor_origin The position of the sensor in the world frame
   * @param laser_return The position of the laser return in the world frame
   */
  void updateMap(ProbabilityMap& map, const gtsam::Point2& sensor_origin, const gtsam::Point2& laser_return) const;

  /**
   * Update the map with a single laser scan message
   * @param map The map to update
   * @param scan The laser scan message to be added to the map
   * @param world_T_base The pose (2D) of the robot/base in the world frame
   * @param base_T_laser The pose (3D) of the laser in the robot/base frame
   */
  void updateMap(ProbabilityMap& map, const sensor_msgs::LaserScan& scan, const gtsam::Pose2& world_T_base, const gtsam::Pose3& base_T_laser) const;

protected:

  double range_sigma_; ///< The measurement uncertainty of the laser
  bool use_max_range_; ///< Use max_range measurements to clear (but not mark) the map
};

} // namespace sensor_models

} // namespace bnr_mapping

#endif // SENSOR_MODELS_H


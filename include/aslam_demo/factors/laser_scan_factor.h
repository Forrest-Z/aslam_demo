/**
 * laser_scan_factor.h
 */

#ifndef LASER_SCAN_FACTOR_H__
#define LASER_SCAN_FACTOR_H__

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose2.h>

namespace factors {

/**
 * A class for a laser scan constraint produced by the Canonical Scan Matcher (ICP). This
 * is basically identical the the standard GTSAM BetweenFactor. A new class has been derived
 * to distinguish between odometry factors and loop closure factors.
 */
class LaserScanFactor: public gtsam::BetweenFactor<gtsam::Pose2> {

public:

  /**
   * shorthand for a smart pointer to a factor
   */
  typedef typename boost::shared_ptr<LaserScanFactor> shared_ptr;

  /** default constructor - only use for serialization */
  LaserScanFactor() {
  }

  virtual ~LaserScanFactor() {
  }

  /**
   * Constructor
   */
  LaserScanFactor(gtsam::Key key1, gtsam::Key key2, const gtsam::Pose2& measurement, const gtsam::noiseModel::Base::shared_ptr& model);

  /**
   * Makes a deep copy of this factor
   * @return a deep copy of this factor
   */
  gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new LaserScanFactor(*this)));
  }

  /**
   * print this factor to stdout
   * @param title
   * @param keyFormatter
   */
  void print(const std::string& title = "Laser Scan Factor:", const gtsam::KeyFormatter& keyFormatter =
      gtsam::DefaultKeyFormatter) const;

  /**
   * Print this factor to the stream
   * @param stream Input stream object
   * @param factor factor to print
   * @return Output stream object
   */
  friend std::ostream& operator<<(std::ostream& stream, const LaserScanFactor& factor);

private:

  /**
   * shorthand for this factor's base class
   */
  typedef typename gtsam::BetweenFactor<gtsam::Pose2> Base;

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & boost::serialization::make_nvp("BetweenFactor", boost::serialization::base_object<Base>(*this));
  }
};

} // namespace factors

#endif // LASER_SCAN_FACTOR_H__


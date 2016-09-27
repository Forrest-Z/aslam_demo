/**
 * odometry_factor.h
 */

#ifndef ODOMETRY_FACTOR_H__
#define ODOMETRY_FACTOR_H__

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose2.h>

namespace bnr_factors {

/**
 * A class for an odometry constraint. This is basically identical the the standard GTSAM
 * BetweenFactor. A new class has been derived to distinguish odometry factors from loop
 * closure factors.
 */
class OdometryFactor: public gtsam::BetweenFactor<gtsam::Pose2> {

public:

  /**
   * shorthand for a smart pointer to a factor
   */
  typedef typename boost::shared_ptr<OdometryFactor> shared_ptr;

  /** default constructor - only use for serialization */
  OdometryFactor() {
  }

  virtual ~OdometryFactor() {
  }

  /**
   * Constructor
   */
  OdometryFactor(gtsam::Key key1, gtsam::Key key2, const gtsam::Pose2& measurement, const gtsam::noiseModel::Base::shared_ptr& model);

  /**
   * Makes a deep copy of this factor
   * @return a deep copy of this factor
   */
  gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new OdometryFactor(*this)));
  }

  /**
   * print this factor to stdout
   * @param title
   * @param keyFormatter
   */
  void print(const std::string& title = "Odometry Factor:", const gtsam::KeyFormatter& keyFormatter =
      gtsam::DefaultKeyFormatter) const;

  /**
   * Print this factor to the stream
   * @param stream Input stream object
   * @param factor Odometry factor to print
   * @return Output stream object
   */
  friend std::ostream& operator<<(std::ostream& stream, const OdometryFactor& factor);

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

} // namespace bnr_factors

#endif // ODOMETRY_FACTOR_H__



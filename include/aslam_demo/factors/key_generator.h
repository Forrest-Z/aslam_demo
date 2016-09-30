/**
 * key_generator.h
 */

#ifndef KEY_GENERATOR_H_
#define KEY_GENERATOR_H_

#include <gtsam/nonlinear/Key.h>
#include <ros/ros.h>

namespace factors {

/**
 * Enum defining known variable types and their corresponding character key
 */
namespace key_type {
  enum Enum {
    Pose2 = 'x', //!< Pose2
    Pose3 = 'X', //!< Pose3
    Point2 = 'p',//!< Point2
    Point3 = 'P',//!< Point3
  };
} /// @namespace key_type

/**
 * A class for generating unique GTSAM variable names (aka Keys or Symbols) from a ROS Time object.
 * The variable name is based only on the provided time and allowed delta. This allows factors to
 * be created in a distributed way; the keys generated from one subsystem will be compatible with
 * the others, even though no communication/common class instance is involved.
 */
class KeyGenerator {

public:

  /**
   * Constructor
   * @param time_delta The maximum time difference that may be considered the same time/key/index/pose
   */
  KeyGenerator(double time_delta = 0.10);

  /**
   * Destructor
   */
  virtual ~KeyGenerator();

  /**
   * Returns the maximum time difference that may be considered the same time/key/index/pose
   */
  double timeDelta() const { return time_delta_nsec_ * 1.0e-9; };

  /**
   * Generate a GTSAM key/symbol from a timestamp and variable type
   * @param key_type
   * @param stamp
   * @return a GTSAM symbol that can be used a unique variable name within a GTSAM factor graph
   */
  gtsam::Key generateKey(const key_type::Enum& key_type, const ros::Time& stamp) const;

  /**
   * Extract the variable type (Pose2, Point2, etc) encoded in the provided key/symbol
   * @param key
   * @return Variable type of provided key
   */
  key_type::Enum extractKeyType(const gtsam::Key& key) const;

  /**
   * Extract the timestamp encoded in the provided key/symbol. Note that the use of a time delta
   * means that the extracted timestamp may not be exactly the same as the original timestamp.
   * @param key
   * @return Timestamp encoded in the provided key
   */
  ros::Time extractTimestamp(const gtsam::Key& key) const;

  /**
   * Compute the quantized timestamp that will be associated with any key created with the
   * provided timestamp
   * @param stamp
   * @return
   */
  ros::Time computeQuantizedTimestamp(const ros::Time& stamp) const;

protected:
  unsigned int time_delta_nsec_; ///< The maximum time difference that may be considered the same time/key/index/pose (in nanoseconds)
};


// Helper function for BNR Key Formatter
std::string _bnrTimestampKeyFormatter(gtsam::Key key);

// Helper function for BNR Key Formatter
std::string _bnrRawKeyFormatter(gtsam::Key key);

///
/// A KeyFormatter that will convert the keys into labeled timestamps
///
static const gtsam::KeyFormatter BnrTimestampKeyFormatter = &_bnrTimestampKeyFormatter;

///
/// A KeyFormatter that will convert the keys into the raw integer
///
static const gtsam::KeyFormatter BnrRawKeyFormatter = &_bnrRawKeyFormatter;

} // namespace factors

#endif // KEY_GENERATOR_H_


/**
 * key_generator.cpp
 */

#include <aslam_demo/factors/key_generator.h>
//#include <gtsam_ros/gtsam_ros.h>

namespace bnr_factors {

/* ************************************************************************* */
KeyGenerator::KeyGenerator(double time_delta) {
  // Determine the max count associated with the specified time delta
  size_t max_count = 1.0 / time_delta;

  // Truncate the max count to an integer, and clip to fit inside a 24-bit counter
  // The smallest supported max_count is 1 (1.0s delta)
  // The largest supported max_count is (1 << 24), the largest number that will fit inside 24-bits (60ns delta)
  max_count = std::min(size_t(1<<24), std::max(size_t(1), max_count));

  // Compute the actual time delta from the max_count value
  time_delta_nsec_ = std::ceil(1.0e+9 / max_count);
}

/* ************************************************************************* */
KeyGenerator::~KeyGenerator() {
}

/* ************************************************************************* */
gtsam::Key KeyGenerator::generateKey(const key_type::Enum& key_type, const ros::Time& stamp) const {
  // A GTSAM Symbol consists of an 8-bit character and a 56-bit index
  // The character is defined by the key type. The actual character is defined in the enum
  // The index is formed by:
  //  (1) Converting the nanosecond field into counts of time_delta_ seconds. This should be less than a 24-bit number.
  //  (2) Stacking the 8-bit character code, the 32-bit seconds field, and the 24-bit time_delta_counts to form a 64-bit value

  // Create the index
  size_t time_delta_counts = stamp.nsec / time_delta_nsec_;
  size_t index = ((size_t)key_type << 56) + ((size_t)stamp.sec << 24) + ((size_t)time_delta_counts);

  // Create the GTSAM Key
  return gtsam::Key(index);
}

/* ************************************************************************* */
key_type::Enum KeyGenerator::extractKeyType(const gtsam::Key& key) const {
  // The key type is encoded in the upper 8-bits. Extract it and convert to the Enum
  return key_type::Enum( (key >> 56) & 0x000000FF );
}

/* ************************************************************************* */
ros::Time KeyGenerator::extractTimestamp(const gtsam::Key& key) const {
  // The seconds are encoded in bits 56->24
  size_t sec = (key >> 24) & 0xFFFFFFFF;

  // The nanoseconds are encoded in the bottom 24-bits, scaled by time_delta_
  size_t nsec = (key & 0x00FFFFFF) * time_delta_nsec_;

  // Create a ROS Time object from the recovered seconds and nanoseconds
  return ros::Time(sec, nsec);
}

/* ************************************************************************* */
ros::Time KeyGenerator::computeQuantizedTimestamp(const ros::Time& stamp) const {
  // Quantize the partial seconds
  size_t time_delta_counts = stamp.nsec / time_delta_nsec_;

  // Return the quantized time
  return ros::Time(stamp.sec, time_delta_nsec_ * time_delta_counts);
}

/* ************************************************************************* */
std::string _bnrTimestampKeyFormatter(gtsam::Key key) {

  char type = (key >> 56) & 0x000000FF;

  // The seconds are encoded in bits 56->24
  // The nanoseconds are encoded in the bottom 24-bits, scaled by time_delta_
  double sec = double((key >> 24) & 0xFFFFFFFF) + (0.001 * double(key & 0x00FFFFFF));

  return std::string() + type + boost::lexical_cast<std::string>(sec) + " [" + boost::lexical_cast<std::string>(key) + "]";
}

/* ************************************************************************* */
std::string _bnrRawKeyFormatter(gtsam::Key key) {
  return boost::lexical_cast<std::string>(key);
}

/* ************************************************************************* */
} /// @namespace bnr_factors


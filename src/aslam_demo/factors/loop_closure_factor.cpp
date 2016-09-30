/**
 * loop_closure_factor.cpp
 */

#include <aslam_demo/factors/loop_closure_factor.h>
#include <gtsam_ros/gtsam_ros.h>

namespace factors {

/* ************************************************************************* */
LoopClosureFactor::LoopClosureFactor(gtsam::Key key1, gtsam::Key key2, const gtsam::Pose2& measurement, const gtsam::noiseModel::Base::shared_ptr& model)
    : Base(key1, key2, measurement, model) {
}

/* ************************************************************************* */
void LoopClosureFactor::print(const std::string& title, const gtsam::KeyFormatter& keyFormatter) const {
  // TODO: This version of print does not use the provided key formatter. The next version of GTSAM
  // is supposed to support the stream operator natively. When we upgrade, follow GTSAM standards here.

  // Implement print using the stream operator
  std::cout << title << std::endl;
  std::cout << *this;
}

/* ************************************************************************* */
std::ostream& operator<<(std::ostream& stream, const LoopClosureFactor& factor) {
  stream << "  Pose Key1: " << gtsam::DefaultKeyFormatter(factor.key1()) << std::endl;
  stream << "  Pose Key2: " << gtsam::DefaultKeyFormatter(factor.key2()) << std::endl;
  stream << "  Measurement: (" << factor.measured().x() << ", " << factor.measured().y() << " | " << factor.measured().theta() << ")" << std::endl;
  stream << "  Noise Model: " << gtsam::ToString(*factor.get_noiseModel()) << std::endl;
  return stream;
}

/* ************************************************************************* */
}
 // namespace factors


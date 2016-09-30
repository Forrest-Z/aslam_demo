/**
 * optimization_processing.h
 */

#ifndef OPTIMIZATION_PROCESSING_H
#define OPTIMIZATION_PROCESSING_H

#include <ros/ros.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Key.h>
#include <gtsam/base/Matrix.h>

namespace mapping {

namespace optimization {

typedef std::map<gtsam::Key, gtsam::Matrix> Covariances;

/**
 * Perform a series of checks on the factor graph before optimization. This checks that the
 * factor graph is connected, all variables have been initialized, etc.
 * @param factors The set of factors to test
 * @param values A set of initial values for each variable
 * @return True if the factor graph passes all checks and can be optimized
 */
bool validateFactorGraph(const gtsam::NonlinearFactorGraph& factors, const gtsam::Values& values);

/**
 * Use GTSAM to optimize the factor graph
 * @param configuration The set of configuration parameters
 * @param factors The set of factors to optimize
 * @param values A set of initial values for each variable
 * @return The optimized set of variables
 */
gtsam::Values optimizeFactorGraph(const gtsam::NonlinearFactorGraph& factors, const gtsam::Values& values, const gtsam::LevenbergMarquardtParams& parameters);

/**
 * Use GTSAM to compute the marginal covariances for each variable.
 * Note that this is time consuming.
 * @param configuration The set of configuration parameters
 * @param factors The set of factors to optimize
 * @param values A set of optimal values for each variable
 * @return The marginal covariance matrix for each variable
 */
Covariances computeCovariances(const gtsam::NonlinearFactorGraph& factors, const gtsam::Values& values);

///**
// * Serialize the pose covariances to a log file for easy MATLAB parsing.
// * Format: Timestamp Covariance (in row-major order, c11 c12 c13 c21 c22 c23 c31 c32 c33)
// * @param filename The filename of the CSV log to create
// * @param time_tolerance
// * @param values The set of values to serialize
// * @param covariances The marginal covariance matrix for each variable
// */
//void writeCovariances(const std::string& filename, double time_tolerance, const gtsam::Values& values, const Covariances& covariances);

} /// @namespace optimization

} /// @namespace bnr_mapping

#endif // OPTIMIZATION_PROCESSING_H


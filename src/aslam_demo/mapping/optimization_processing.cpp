/**
 * optimization_processing.cpp
 */

#include <aslam_demo/mapping/optimization_processing.h>
#include <aslam_demo/mapping/mapping_common.h>
#include <aslam_demo/mapping/timer.h>
#include <aslam_demo/factors/odometry_factor.h>
#include <aslam_demo/factors/laser_scan_factor.h>
#include <aslam_demo/factors/loop_closure_factor.h>
#include <aslam_demo/factors/key_generator.h>
#include <gtsam/nonlinear/Marginals.h>
#include <boost/filesystem.hpp>
#include <queue>
#include <fstream>

namespace mapping {

namespace optimization {

/* ************************************************************************* */
bool validateFactorGraph(const gtsam::NonlinearFactorGraph& factors, const gtsam::Values& values) {

  // Extract all of the keys from the factor graph and values
  gtsam::FastSet<gtsam::Key> factor_keys = factors.keys();
  gtsam::FastList<gtsam::Key> values_keys_list = values.keys();
  gtsam::FastSet<gtsam::Key> values_keys(values_keys_list.begin(), values_keys_list.end());

  // Check that all factor graph keys are contained in the values
  std::vector<gtsam::Key> missing_values;
  std::set_difference(factor_keys.begin(), factor_keys.end(), values_keys.begin(), values_keys.end(), std::back_inserter(missing_values));
  bool all_keys_have_values = missing_values.empty();
  for(size_t i = 0; i < missing_values.size(); ++i) {
    ROS_ERROR_STREAM("Key " << factors::BnrTimestampKeyFormatter(missing_values.at(i)) << " does not have a value.");
  }

  // Check that all keys in the values structure are involved in at least one factor
  std::vector<gtsam::Key> missing_factors;
  std::set_difference(values_keys.begin(), values_keys.end(), factor_keys.begin(), factor_keys.end(), std::back_inserter(missing_factors));
  bool all_keys_have_factors = missing_factors.empty();
  for(size_t i = 0; i < missing_factors.size(); ++i) {
    ROS_ERROR_STREAM("Key " << factors::BnrTimestampKeyFormatter(missing_factors.at(i)) << " does not have a factor.");
  }

  // Check that the graph is connected. This is done via a breadth-first search
  bool graph_is_connected = false;
  {
    // Convert the factor graph to a "symbolic" version with sequentially numbered keys
    gtsam::Ordering::shared_ptr ordering = values.orderingArbitrary();
    gtsam::SymbolicFactorGraph::shared_ptr symbolic = factors.symbolic(*ordering);
    // Compute the variable adjacency matrix
    std::vector<std::set<gtsam::Index> > adjacency(ordering->size());
    for(size_t factor_index = 0; factor_index < symbolic->size(); ++factor_index) {
      if(symbolic->operator[](factor_index)) {
        BOOST_FOREACH(const gtsam::Index key1, symbolic->operator[](factor_index)->keys()) {
          BOOST_FOREACH(const gtsam::Index key2, symbolic->operator[](factor_index)->keys()) {
            if(key1 != key2) adjacency[key1].insert(key2);
          }
        }
      }
    }
    // Create a list of visited keys
    std::vector<gtsam::Index> visited(values_keys.size(), false);
    // Create a queue of pending keys to check
    std::set<gtsam::Index> queue;
    // Push the first key into the queue to start the search
    if(!visited.empty()) queue.insert(0);
    // Loop until we are out of keys in the queue
    while(!queue.empty()) {
      // Pop the next key off of the queue
      gtsam::Index key = *queue.begin();
      queue.erase(queue.begin());
      // Mark the key as visited
      visited.at(key) = true;
      // Add all unvisited neighbors to the queue
      BOOST_FOREACH(gtsam::Index adjacent_key, adjacency[key]) {
        if(!visited[adjacent_key]) {
          queue.insert(adjacent_key);
        }
      }
    }
    // Check for unvisited keys
    std::vector<gtsam::Key> unvisited_keys;
    for(size_t i = 0; i < visited.size(); ++i) {
      if(!visited[i]) {
        unvisited_keys.push_back(ordering->key(i));
      }
    }
    if(unvisited_keys.size() < 10) {
      for(size_t i = 0; i < unvisited_keys.size(); ++i) {
        ROS_ERROR_STREAM("Key " << factors::BnrTimestampKeyFormatter(unvisited_keys[i]) << " is not connected.");
      }
    } else if(unvisited_keys.size() >= 10) {
      for(size_t i = 0; i < 5; ++i) {
        ROS_ERROR_STREAM("Key " << factors::BnrTimestampKeyFormatter(unvisited_keys[i]) << " is not connected.");
      }
      ROS_ERROR_STREAM((unvisited_keys.size() - 5) << " additional keys are not connected.");
    }
    graph_is_connected = (unvisited_keys.empty());
  }

  return graph_is_connected && all_keys_have_values && all_keys_have_factors;
}

/* ************************************************************************* */
void customOptimizationLoop(gtsam::LevenbergMarquardtOptimizer& optimizer) {
  const gtsam::LevenbergMarquardtParams& params = optimizer.params();
  double currentError = optimizer.error();

  // check if we're already close enough
  if(currentError <= params.errorTol) {
    if (params.verbosity >= gtsam::NonlinearOptimizerParams::ERROR)
      std::cout << "Exiting, as error = " << currentError << " < " << params.errorTol << std::endl;
    return;
  }

  // Maybe show output
  if (params.verbosity >= gtsam::NonlinearOptimizerParams::VALUES) optimizer.values().print("Initial values");
  if (params.verbosity >= gtsam::NonlinearOptimizerParams::ERROR) std::cout << "Initial error: " << currentError << std::endl;

  // Return if we already have too many iterations
  if(optimizer.iterations() >= params.maxIterations)
    return;

  // Iterative loop
  do {
    // Check the lambda value. If it is too small, hold it to a reasonable value
    // This is the entire reason a new optimization loop was written: the lambda
    // value got so small, it went to zero even with double precision. When attempting
    // to increase the lambda value later, it was stuck at zero.
    if(optimizer.lambda() < 1.0e-20) optimizer.state().lambda = 1.0e-20;

    // Do next iteration
    currentError = optimizer.error();
    optimizer.iterate();

    // Maybe show output
    if(params.verbosity >= gtsam::NonlinearOptimizerParams::VALUES) optimizer.values().print("newValues");
    if(params.verbosity >= gtsam::NonlinearOptimizerParams::ERROR) std::cout << "newError: " << optimizer.error() << std::endl;
  } while(optimizer.iterations() < params.maxIterations &&
      !gtsam::checkConvergence(params.relativeErrorTol, params.absoluteErrorTol,
            params.errorTol, currentError, optimizer.error(), params.verbosity));

  // Printing if verbose
  if (params.verbosity >= gtsam::NonlinearOptimizerParams::ERROR &&
      optimizer.iterations() >= params.maxIterations)
    std::cout << "Terminating because reached maximum iterations" << std::endl;
}

/* ************************************************************************* */
gtsam::Values optimizeFactorGraph(const gtsam::NonlinearFactorGraph& factors, const gtsam::Values& values, const gtsam::LevenbergMarquardtParams& parameters) {
  gtsam::Values optimized_values;

  Timer timer;
  timer.start();

  // Create the optimizer, optimize the factor graph, and extract the optimized values
  double initial_error = 0;
  double final_error = 0;
  try{
    gtsam::LevenbergMarquardtOptimizer optimizer(factors, values, parameters);
    initial_error = optimizer.error();
    // Use the custom optimization loop instead of the built-in 'optimizer.optimize()'. The built-in function
    // does not have a lower-bound on lambda, which can cause an infinite loop in rare situations.
    customOptimizationLoop(optimizer);
    optimized_values = optimizer.values();
    final_error = optimizer.error();
  } catch(const std::exception& e) {
    throw std::runtime_error("An error occurred while optimizing the factor graph: " + std::string(e.what()));
  }

  timer.stop();
  ROS_DEBUG_STREAM("Optimized " << factors.size() << " factors in " << timer.elapsed() << " seconds.");
  ROS_DEBUG_STREAM("Initial Error = " << initial_error << ", Final Error = " << final_error);

  return optimized_values;
}

/* ************************************************************************* */
Covariances computeCovariances(const gtsam::NonlinearFactorGraph& factors, const gtsam::Values& values) {
  Covariances covariances;

  // Loop over all of the variables, using the GTSAM "Marginals" object
  // to compute the marginal covariance for each variable
  try {
    // Compute the marginal covariance on each variable
    gtsam::Marginals marginals(factors, values, gtsam::Marginals::QR);
    BOOST_FOREACH(const gtsam::Values::ConstKeyValuePair& key_value, values) {
      covariances[key_value.key] = marginals.marginalCovariance(key_value.key);
    }
  } catch(const std::exception& e) {
    throw std::runtime_error("An error occurred while computing marginal covariances for each key: " + std::string(e.what()));
  }

  return covariances;
}

///* ************************************************************************* */
//void writeCovariances(const std::string& filename, double time_tolerance, const gtsam::Values& values, const Covariances& covariances) {
//
//  // create an output file
//  boost::filesystem::path file_path(filename);
//  boost::filesystem::create_directories(file_path.parent_path());
//  std::ofstream logfile(filename.c_str());
//
//  // Loop through each variable, writing the output to disk
//  factors::KeyGenerator key_generator(time_tolerance);
//  BOOST_FOREACH(const gtsam::Values::ConstKeyValuePair& key_value, values) {
//    const gtsam::Key& key = key_value.key;
//    ros::Time time = key_generator.extractTimestamp(key);
//    factors::key_type::Enum type = key_generator.extractKeyType(key);
//    if(type == factors::key_type::Pose2) {
//      logfile << time;
//      // Look for the associated covariance
//      Covariances::const_iterator covariance_iter = covariances.find(key);
//      if(covariance_iter != covariances.end()) {
//        const gtsam::Matrix& cov = covariance_iter->second;
//        logfile << " " << cov(0,0) << " " << cov(0,1) << " " << cov(0,2)
//                << " " << cov(1,0) << " " << cov(1,1) << " " << cov(1,2)
//                << " " << cov(2,0) << " " << cov(2,1) << " " << cov(2,2);
//      }
//      logfile << std::endl;
//    }
//  }
//
//  // Close the logfile
//  logfile.close();
//}

/* ************************************************************************* */
} /// @namespace optimization

} /// @namespace bnr_mapping


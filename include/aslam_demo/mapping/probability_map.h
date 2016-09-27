/**
 * probability_map.h
 */

#ifndef PROBABILITY_MAP_H
#define PROBABILITY_MAP_H

#include <gtsam/geometry/Point2.h>
#include <gtsam/base/Matrix.h>
#include <boost/shared_ptr.hpp>

namespace bnr_mapping {

/**
 * A class for maintaining a floating-point occupancy grid. ROS currently supports
 * a 3-level map (occupancy_grid) and a 256-level map (costmap2d), but no floating-point
 * version. This is needed to properly represent probabilities. Internally, the grid
 * cells store the log-odds that the cell is occupied, but the user interacts with the
 * map using standard probabilities.
 */
class ProbabilityMap {
public:

	/**
	 * Convenience typedef for shared pointer to this object
	 */
	typedef boost::shared_ptr<ProbabilityMap> shared_ptr;

	/**
	 * Constructor that generates an empty map of a given size
	 * @param rows The number of rows in the map
	 * @param cols The number of columns in the map
	 * @param cell_size The width and height of each cell/pixel of the map in meters
	 * @param origin The position in map coordinates of the lower-left corner of the image (map_server compatibility)
	 */
	ProbabilityMap(size_t rows = 1, size_t cols = 1, double cell_size = 1.0,
			const gtsam::Point2& origin = gtsam::Point2(0.0, 0.0));

	/**
	 * Destructor
	 */
	~ProbabilityMap();

	/**
	 * GTSAM-style print function that writes to stdout
	 */
	void print(const std::string& name = "Probability Map:\n") const;

	/**
	 * Print the occupancy map to the stream
	 * @param stream Input stream object
	 * @param map Occupancy map to print
	 * @return Output stream object
	 */
	friend std::ostream& operator<<(std::ostream& stream,
			const ProbabilityMap& map);

	/**
	 * GTSAM-style equals function for comparing two maps
	 * @param rhs The other map to compare with (right hand side of equality)
	 * @param tol The tolerance used when comparing floating-point values
	 * @return true if the two maps are equivalent up to tol
	 */
	bool equals(const ProbabilityMap& rhs, double tol = 1e-9) const;

	/**
	 * Return the position in map coordinates of the world coordinate origin
	 * @return The position in map coordinates of the world coordinate origin
	 */
	const gtsam::Point2& origin() const {
		return origin_;
	}

	/**
	 * Return the size of each cell/pixel in meters
	 * @return The size of each cell/pixel in meters
	 */
	double cellSize() const {
		return cell_size_;
	}

	/**
	 *
	 * @return
	 */
	size_t rows() const {
	  return data_.rows();
	}

  /**
   *
   * @return
   */
  size_t cols() const {
    return data_.cols();
  }

  /**
   * Load data into the map, overwriting any existing data. The data array must be the proper size.
   * @param data An array of floats containing the proper number of elements to load into the map
   */
  void load(double* data);

  /**
   * Clear the map, setting every cell to unknown
   */
  void clear();

  /**
   * Convert map cell/pixel coordinates to world coordinates (in meters)
   * This transformation is derived from the cell size and origin
   * @param map_coordinates 2D coordinates specifying a map cell/pixel
   * @return 2D world coordinates in meters
   */
  gtsam::Point2 toWorld(const gtsam::Point2& map_coordinates) const;

  /**
   * Convert world coordinates (in meters) to map cell/pixel coordinates
   * This transformation is derived from the cell size and origin
   * @param world_coordinates 2D coordinates specifying a world position (in meters)
   * @return 2D map cell/pixel coordinates
   */
  gtsam::Point2 fromWorld(const gtsam::Point2& world_coordinates) const;

  /**
   * Test if a (row, col) pair is inside the map area
   * @param row
   * @param col
   * @return True if the (row, col) pair is inside the map
   */
  bool inside(int row, int col) const;

  /**
   * Test if a cell index (row-major) is inside the map area
   * @param index
   * @return True if the index is inside the map
   */
  bool inside(size_t index) const {
    return inside( index / cols(), index % cols() );
  }

  /**
   * Test if a map coordinate is inside the map area
   * @param map_coordinates
   * @return True if the map point (x,y) is inside the map
   */
  bool inside(const gtsam::Point2& map_coordinates) const {
    return inside(std::floor(map_coordinates.y()), std::floor(map_coordinates.x()));
  }

  /**
   * Return the obstacle probability of the map by cell address
   * @param row
   * @param col
   * @return obstacle probability at (row, col)
   */
  double at(int row, int col) const;

  /**
   * Return the obstacle probability of the map by cell index (row-major)
   * @param index
   * @return obstacle probability at [index]
   */
  double at(size_t index) const {
    return at( index / cols(), index % cols() );
  }

  /**
   * Return the obstacle probability of the map by (x,y) point.
   * This version rounds the (x,y) point to an integer address
   * @param map_coordinates
   * @return obstacle probability at (x,y)
   */
  double at(const gtsam::Point2& map_coordinates) const {
    return at(std::floor(map_coordinates.y()), std::floor(map_coordinates.x()));
  }

  /**
   * Return the sub-pixel obstacle probability of the map by (x,y) point.
   * This version interpolates the (x,y) point from the surrounding map cells
   * @param map_coordinates
   * @return obstacle probability at (x,y)
   */
  double interpolate(const gtsam::Point2& map_coordinates) const;

  /**
   * Find the intersection of a ray with a bounding box
   * @param start_point Starting point of the ray in world coordinates
   * @param end_point End point of the ray in world coordinates
   * @param box_min Lower-left corner of the bounding box in world coordinates
   * @param box_max Upper-right corner of the bounding box in world coordinates
   * @return
   */
  std::pair<gtsam::Point2,gtsam::Point2> findIntersections(const gtsam::Point2& start_point, const gtsam::Point2& end_point,
      const gtsam::Point2& lower_left, const gtsam::Point2& upper_right) const;

  /**
   * Extract all of the points in the map above the supplied threshold.
   * @param threshold
   * @return The map coordinates of cells with a probability higher than 'threshold'
   */
  std::vector<gtsam::Point2> points(double threshold = 0.0) const;

  typedef struct {
    int row; ///< Map coordinates of a Bresenham cell
    int col; ///< Map coordinates of a Bresenham cell
    gtsam::Point2 start; ///< World coordinates of the entry point into this cell
    gtsam::Point2 end; ///< World coordinates of the exit point out of this cell
  } LineCell;

	/**
	 * Return a container of map points that exist along the line from start_point to end_point.
	 * If the line extends off the map on either side, just the section of the
	 * line within the map boundaries is returned.
	 * @param start_point in world coordinates
	 * @param end_point in world coordinates
	 * @return A container of row,col pairs
	 */
	std::vector<LineCell> line(const gtsam::Point2& start_point, const gtsam::Point2& end_point) const;

	/**
	 * Incrementally update a map cell with a new observation probability
	 * @param row
	 * @param col
	 * @param probability
	 */
	void update(int row, int col, double probability);

  /**
   * Incrementally update a map index with a new observation probability
   * @param index
   * @param probability
   */
  void update(size_t index, double probability) {
    update(index / cols(), index % cols(), probability);
  }

  /**
   * Incrementally update a map point with a new observation probability
   * This version rounds the (x,y) point to an integer address
   * @param map_coordinates
   * @param probability
   */
  void update(const gtsam::Point2& map_coordinates, double probability) {
    update(std::floor(map_coordinates.y()), std::floor(map_coordinates.x()), probability);
  }

	/**
	 * Create an Occupancy Grid version of the probability map. An occupancy grid represents the
	 * probability that a cell is occupied. The function returns the probability in the range [0 255]
	 * @return 8-bit occupancy grid
	 */
  gtsam::Matrix occupancyGrid() const;

  /**
   * Save an Occupancy Grid version of the probability map to disk in PGM format. An occupancy
   * grid represents the probability that a cell is occupied. At the moment, only the PGM image
   * format is supported. A YAML map description file is also produced.
   */
  void occupancyGrid(const std::string& filename) const;

  /**
   * Blur/smooth the map values
   * @param sigma
   */
  void smooth(double sigma);

protected:

	/**
	 * Storage for the map data
	 */
	gtsam::Matrix data_;

	/**
	 * The map coordinates of the world frame origin of the map
	 */
	gtsam::Point2 origin_;

  /**
   * The size of each map cell/pixel in meters
   */
  double cell_size_;

	/**
	 * Maximum allowable log-odds magnitude to be stored in the map. Larger values
	 * will be clipped to prevent the map from becoming too confident and allow
	 * cells to switch designations faster in the presence of dynamic map elements
	 * (i.e. people)
	 */
	static const double MAX_LOG_ODDS;

	/**
	 * Convert a Log-Odds value into a probability
	 * @param log_odds
	 * @return probability
	 */
	static double LogOddsToProbability(double log_odds);

  /**
   * Convert a probability value into a Log-Odds
   * @param probability
   * @return log_odds
   */
  static double ProbabilityToLogOdds(double probability);

private:

	/**
	 * Serialization function
	 */
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version) {
		ar & BOOST_SERIALIZATION_NVP(data_);
		ar & BOOST_SERIALIZATION_NVP(origin_);
    ar & BOOST_SERIALIZATION_NVP(cell_size_);
	}

	template <typename Derived1, typename Derived2>
	Derived1 conv2d(const Eigen::MatrixBase<Derived1>& input, const Eigen::MatrixBase<Derived2>& kernel);
};

/// @todo: this needs some serious work. conv2d(matrix, vector.transpose()) fails to compile
template <typename Derived1, typename Derived2>
Derived1 ProbabilityMap::conv2d(const Eigen::MatrixBase<Derived1>& input, const Eigen::MatrixBase<Derived2>& kernel) {
  Derived1 output = Derived1::Zero(input.rows(),input.cols());

  typedef typename Derived1::Scalar Scalar1;
  typedef typename Derived2::Scalar Scalar2;

  int col=0,row=0;
  int KSizeX = kernel.rows();
  int KSizeY = kernel.cols();

  int limitRow = input.rows()-KSizeX;
  int limitCol = input.cols()-KSizeY;

  Derived2 block;
  Scalar1 normalization = kernel.sum();
  if(normalization < 1E-6) {
      normalization=1;
  }
  for(row = KSizeX; row < limitRow; row++) {
    for (col = KSizeY; col < limitCol; col++) {
      Scalar1 b = (static_cast<Derived2>(input.block(row,col,KSizeX,KSizeY)).cwiseProduct(kernel)).sum();
      output.coeffRef(row,col) = b;
    }
  }

  return output/normalization;
}

} // namespace bnr_mapping

#endif // PROBABILITY_MAP_H


/**
 * probability_map.cpp
 */

#include <aslam_demo/mapping/probability_map.h>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <fstream>
#include <exception>

namespace mapping {

const double ProbabilityMap::MAX_LOG_ODDS = 50.0;

/* ************************************************************************* */
ProbabilityMap::ProbabilityMap(size_t rows, size_t cols, double cell_size, const gtsam::Point2& origin)
	: data_(gtsam::Matrix::Zero(rows, cols)), origin_(origin), cell_size_(cell_size) {
}

ProbabilityMap::ProbabilityMap(nav_msgs::OccupancyGrid& occupancy_grid) {
  setfromOccupancyGrid(occupancy_grid);
}

/* ************************************************************************* */
ProbabilityMap::~ProbabilityMap() {
}

/* ************************************************************************* */
std::ostream& operator<< (std::ostream& stream, const ProbabilityMap& map) {
	stream << "  cell size: " << map.cellSize() << "\n";
	stream << "  origin: ( " << map.origin().x() << " , " << map.origin().y() << " )\n";
	for(size_t i = 0; i < map.rows(); ++i) {
		if(i == 0) {
			stream << "  data:";
		} else {
			stream << "       ";
		}
		for(size_t j = 0; j < map.cols(); ++j) {
			stream << " " << map.at(i,j);
		}
		stream << "\n";
	}
	stream << std::endl;
	return stream;
}


void ProbabilityMap::setfromOccupancyGrid(nav_msgs::OccupancyGrid& occupancy_grid) {
  origin_ = gtsam::Point2(occupancy_grid.info.origin.position.x,occupancy_grid.info.origin.position.y);
  cell_size_ = occupancy_grid.info.resolution;
  data_ = gtsam::Matrix::Zero(occupancy_grid.info.height,occupancy_grid.info.width);
  for(size_t row = 0;row < rows();row++)
    for(size_t col = 0;col < cols();col++) {
      data_(row,col) = ProbabilityToLogOdds((255.0 - (double)(occupancy_grid.data[row*cols() + col]))/255.0);
    }
}

/* ************************************************************************* */
void ProbabilityMap::print(const std::string& name) const {
	// Implement print using the stream operator
	std::cout << name << std::endl;
	std::cout << *this;
}

/* ************************************************************************* */
bool ProbabilityMap::equals(const ProbabilityMap& rhs, double tol) const {
  return origin_.equals(rhs.origin_, tol)
		     && (std::fabs(cell_size_ - rhs.cell_size_) < tol)
		     && gtsam::equal_with_abs_tol(data_, rhs.data_, tol);
}

/* ************************************************************************* */
void ProbabilityMap::load(double* data) {
	// Copy data into map
  data_ = gtsam::Matrix_(rows(), cols(), data);
}

/* ************************************************************************* */
void ProbabilityMap::clear() {
  data_ = gtsam::Matrix::Zero(rows(), cols());
}

/* ************************************************************************* */
gtsam::Point2 ProbabilityMap::toWorld(const gtsam::Point2& map_coordinates) const {
	return cell_size_ * map_coordinates + origin_;
}

/* ************************************************************************* */
gtsam::Point2 ProbabilityMap::fromWorld(const gtsam::Point2& world_coordinates) const {
	return (world_coordinates - origin_) / cell_size_;
}

/* ************************************************************************* */
bool ProbabilityMap::inside(int row, int col) const {
  return    (row >= 0)
         && (row < rows())
         && (col >= 0)
         && (col < cols());
}

/* ************************************************************************* */
double ProbabilityMap::at(int row, int col) const {
  // Bounds check
  if(!inside(row,col)) throw std::runtime_error("Requested map coordinates ("
      + boost::lexical_cast<std::string>(row) + "," + boost::lexical_cast<std::string>(col)
      + ") is not within the map bounds.");

  // Convert the log-odds entry into probability
  return LogOddsToProbability(data_(row,col));
}

/* ************************************************************************* */
double ProbabilityMap::interpolate(const gtsam::Point2& map_coordinates) const {

  // Bounds check
  if(!inside(map_coordinates)) throw std::runtime_error("Requested map coordinates ("
      + boost::lexical_cast<std::string>(map_coordinates.y()) + "," + boost::lexical_cast<std::string>(map_coordinates.x())
      + ") is not within the map bounds.");

	// Interpolate the pixel value from the neighbors
  int x1, x2, y1, y2;
  x1 = std::floor(map_coordinates.x());
  if(x1 >= cols()-1) {
    x1 = cols() - 2;
    x2 = cols() - 1;
  } else {
    x2 = x1 + 1;
  }
  y1 = std::floor(map_coordinates.y());
  if(y1 >= rows()-1) {
    y1 = rows() - 2;
    y2 = rows() - 1;
  } else {
    y2 = y1 + 1;
  }

  double dx21 = double(x2 - x1);
  double dx2p = double(x2) - map_coordinates.x();
  double dxp1 = map_coordinates.x() - double(x1);

  double dy21 = double(y2 - y1);
  double dy2p = double(y2) - map_coordinates.y();
  double dyp1 = map_coordinates.y() - double(y1);

  double R1 = (dx2p/dx21)*at(y1, x1) + (dxp1/dx21)*at(y1, x2);
  double R2 = (dx2p/dx21)*at(y2, x1) + (dxp1/dx21)*at(y2, x2);

  return      (dy2p/dy21)*R1            + (dyp1/dy21)*R2;
}

/* ************************************************************************* */
std::pair<gtsam::Point2,gtsam::Point2> ProbabilityMap::findIntersections(const gtsam::Point2& start_point, const gtsam::Point2& end_point,
    const gtsam::Point2& lower_left, const gtsam::Point2& upper_right) const {
  // Source: http://gamedev.stackexchange.com/questions/18436/most-efficient-aabb-vs-ray-collision-algorithms

  // Compute the unit vector for the ray
  gtsam::Point2 direction = start_point.between(end_point).unit();
  // Compute the inverse
  gtsam::Point2 direction_inverse(1.0/direction.x(), 1.0/direction.y());
  // Compute the four possible ray scalars that intersect with the box
  double t1 = (lower_left.x()  - start_point.x())*direction_inverse.x();
  double t2 = (upper_right.x() - start_point.x())*direction_inverse.x();
  double t3 = (lower_left.y()  - start_point.y())*direction_inverse.y();
  double t4 = (upper_right.y() - start_point.y())*direction_inverse.y();
  // Compute the correct min and max ray scalars
  double tmin = std::max(std::min(t1, t2), std::min(t3, t4));
  double tmax = std::min(std::max(t1, t2), std::max(t3, t4));
  /// @todo: If tmax < 0, ray is behind the AABB. This should not happen.
  /// @todo: if tmin > tmax, ray doesn't intersect AABB. This should not happen.
  // Return the two points
  return std::pair<gtsam::Point2,gtsam::Point2>(start_point + tmin*direction, start_point + tmax*direction);
}

/* ************************************************************************* */
std::vector<gtsam::Point2> ProbabilityMap::points(double threshold) const {
  std::vector<gtsam::Point2> points;

  // COnvert the threshold from a probability to a log-odds metric
  double log_odds_threshold = ProbabilityToLogOdds(threshold);

  // Loop over the map, adding points above the log-odds threshold
  for(size_t row = 0; row < rows(); ++row) {
    for(size_t col = 0; col < cols(); ++col) {
      if(data_(row, col) > log_odds_threshold) {
        points.push_back( gtsam::Point2(col, row) );
      }
    }
  }

  return points;
}

/* ************************************************************************* */
void ProbabilityMap::smooth(double sigma) {
  // Convert real-world sigma into the map equivalent
  double map_sigma = sigma / cell_size_;

  // Compute a gaussian smoothing kernel
  size_t kernel_length = 2*std::floor(3.0*map_sigma) + 1;
  Eigen::VectorXd kernel_1d(kernel_length);
  for(size_t i = 0; i < kernel_length; ++i) {
    double x = i - (kernel_length - 1)/2;
    kernel_1d(i) = 1.0/(sigma * std::sqrt(2*M_PI)) * std::exp( (x*x)/(sigma*sigma) );
  }

  // Convolve the kernel with the map (once in each direction)
  data_ = conv2d(data_, kernel_1d);
  kernel_1d.transposeInPlace();
  data_ = conv2d(data_, kernel_1d);
}

gtsam::Point2 ProbabilityMap::findEndPoints(const gtsam::Point2& start_point, double length, double angle) {
  return (gtsam::Point2(start_point.x()+length*cos(angle),start_point.y()+length*sin(angle)));
}


/* ************************************************************************* */
std::vector<ProbabilityMap::LineCell> ProbabilityMap::line(const gtsam::Point2& start_point_world, const gtsam::Point2& end_point_world) const {

  // Midpoint Algorithm
  // (1) Based on the slope, determine if we are incrementing in the +/-X direction or the +/-Y direction
  // (2) Compute the stopping criteria (end pixel)
  // (3) Loop until the entire line has been consumed
  //     (a) Round the current X,Y point to integer coordinates
  //     (b) Compute metadata and add to the output rasterized line
  //     (c) Check for exit conditions
  //     (d) Compute the next (X,Y) position along the line

  std::vector<LineCell> cells;

  // (1) Create a line increment based on the dominate slope direction
  // Convert start and end points into map coordinates
  gtsam::Point2 start_point_map = this->fromWorld(start_point_world);
  gtsam::Point2 end_point_map = this->fromWorld(end_point_world);

  // Compute deltas
  double dx = std::fabs(end_point_map.x() - start_point_map.x());
  int sx = start_point_map.x() < end_point_map.x() ? 1 : -1; // sign of dx
  double dy = std::fabs(end_point_map.y() - start_point_map.y());
  int sy = start_point_map.y() < end_point_map.y() ? 1 : -1; // sign of dy

  // Create a line increment based on the dominate slope direction
  // Grab the total distance along the dominate direction as well
  gtsam::Point2 delta;
  double increments_remaining;
  if(dx > dy) {
    delta = gtsam::Point2(sx, sy*(dy/dx));
    increments_remaining = dx;
  } else {
    delta = gtsam::Point2(sx*(dx/dy), sy);
    increments_remaining = dy;
  }

  // (3) Loop until the entire line has been rasterized
  gtsam::Point2 point = start_point_map;
  while(true) {
    // (a) Round the current line point to integer coordinates
    double u = std::floor(point.x());
    double v = std::floor(point.y());

    // (b) Compute metadata and add to the output rasterized line
    if(inside(v,u)) {
      gtsam::Point2 box_min = toWorld(gtsam::Point2(u,v));
      gtsam::Point2 box_max = toWorld(gtsam::Point2(u+1,v+1));
      std::pair<gtsam::Point2,gtsam::Point2> intersections = findIntersections(start_point_world, end_point_world, box_min, box_max);
      LineCell cell;
      cell.row = v;
      cell.col = u;
      cell.start = intersections.first;
      cell.end = intersections.second;
      cells.push_back(cell);
    }

    // (c) Check if the end point has been reached
    if(increments_remaining <= 0) break;

    // (d) Compute the next (X,Y) position along the line
    double increment;
    if(increments_remaining < 1.0) {
      increment = increments_remaining;
    } else {
      increment = 1.0;
    }
    point += increment*delta;
    increments_remaining -= increment;
  }

  return cells;
}

// Originally used the standard Bresenham Line algorithm. However, due to floating point
// start and end locations, the resulting rasterized line would contain pixels that the
// real line did not intersect with. When computing the entry and exit point in the cell
// metadata, having pixels that do not intersect the line causes issues. There is probably
// a way to modify the algorithm to handle floating point start and end points, but my
// initial attempts resulted in rasterized lines that did not necessarily hit the end point.
///* ************************************************************************* */
//std::vector<ProbabilityMap::LineCell> ProbabilityMap::line(const gtsam::Point2& start_point, const gtsam::Point2& end_point) const {
//// Bresenham's Line Algorithm
//// Pseudocode from wikipedia: http://en.wikipedia.org/wiki/Bresenham's_line_algorithm
////  function line(x0, x1, y0, y1)
////       real deltax := x1 - x0
////       real deltay := y1 - y0
////       real error := 0
////       real deltaerr := abs (deltay / deltax)    // Assume deltax != 0 (line is not vertical),
////             // note that this division needs to be done in a way that preserves the fractional part
////       int y := y0
////       for x from x0 to x1
////           plot(x,y)
////           error := error + deltaerr
////           while error ≥ 0.5 then
////               plot(x, y)
////               y := y + sign(y1 - y0)
////               error := error - 1.0
//
//  std::vector<LineCell> cells;
//
//  // Convert start and end points into map coordinates
//  gtsam::Point2 start_point_map = this->fromWorld(start_point);
//  gtsam::Point2 end_point_map = this->fromWorld(end_point);
//
//  // Convert the start and end point into integer cells
//  int x1 = std::floor(start_point_map.x());
//  int y1 = std::floor(start_point_map.y());
//  int x2 = std::floor(end_point_map.x());
//  int y2 = std::floor(end_point_map.y());
//
//  // Compute deltas
//  int dx = std::abs(x2 - x1);
//  int sx = x1 < x2 ? 1 : -1; // sign of dx
//
//  int dy = std::abs(y2 - y1);
//  int sy = y1 < y2 ? 1 : -1; // sign of dy
//
//  // Initialize the error
//  int error = (dx > dy ? dx : -dy)/2;
//
//  // Initialize the starting point (map)
//  int x = x1;
//  int y = y1;
//
//  // Loop until the end point is found
//  while(true) {
//    // Push the current point into the container
//    // if it is inside the mapping area
//    if(inside(y,x)) {
//      gtsam::Point2 box_min = toWorld(gtsam::Point2(x,y));
//      gtsam::Point2 box_max = toWorld(gtsam::Point2(x+1,y+1));
//      std::pair<gtsam::Point2,gtsam::Point2> intersections = findIntersections(start_point, end_point, box_min, box_max);
//      LineCell cell;
//      cell.row = y;
//      cell.col = x;
//      cell.start = intersections.first;
//      cell.end = intersections.second;
//      cells.push_back(cell);
//    }
//
//    // Check for the stopping criteria
//    if((x == x2) && (y == y2)) { break; }
//
//    // Advance the point along the line
//    int current_error = error;
//    if (current_error > -dx) { error -= dy; x += sx; }
//    if (current_error < +dy) { error += dx; y += sy; }
//  }
//
//  return cells;
//}

/* ************************************************************************* */
void ProbabilityMap::update(int row, int col, double probability) {
  // Bounds check
  if(!inside(row,col)) throw std::runtime_error("Requested map coordinates ("
      + boost::lexical_cast<std::string>(row) + "," + boost::lexical_cast<std::string>(col)
      + ") is not within the map bounds.");

  // Increment the log-odds entry by the provided probability value
  data_(row,col) += ProbabilityToLogOdds(probability);
  if(data_(row,col) > +MAX_LOG_ODDS) data_(row,col) = +MAX_LOG_ODDS;
  if(data_(row,col) < -MAX_LOG_ODDS) data_(row,col) = -MAX_LOG_ODDS;
}

/* ************************************************************************* */
gtsam::Matrix ProbabilityMap::occupancyGrid() const {
  gtsam::Matrix occupancy = gtsam::Matrix::Zero(rows(), cols());

  for(size_t row = 0; row < rows(); ++row) {
    for(size_t col = 0; col < cols(); ++col) {
      occupancy(row,col) = (int)(255 - 255.0*LogOddsToProbability(data_(row,col)));
    }
  }

  return occupancy;
}

void ProbabilityMap::occupancyGrid(nav_msgs::OccupancyGrid& occupancy_msg) {

	std::cout<<"Occ Entered"<<std::endl;
	gtsam::Matrix occupancy = occupancyGrid();

	occupancy_msg.header.frame_id = "map";
	occupancy_msg.header.stamp = ros::Time::now();
	occupancy_msg.header.seq = 1;
	std::cout<<"Header Done"<<std::endl;



	occupancy_msg.info.height = rows();
	occupancy_msg.info.width = cols();
	occupancy_msg.info.resolution = cell_size_;
	occupancy_msg.info.origin.position.x = origin_.x();
	occupancy_msg.info.origin.position.y = origin_.y();
	occupancy_msg.info.origin.position.z =  0.0;
	std::cout<<"Info Done"<<std::endl;

	occupancy_msg.data.resize(rows()*cols());
	int index = 0;
	for(size_t row = 0; row < rows(); ++row) {
	    for(size_t col = 0; col < cols(); ++col) {
	      occupancy_msg.data[index++] = occupancy(row,col);
	    }
	}

}


/* ************************************************************************* */
void ProbabilityMap::occupancyGrid(const std::string& filename) const {

  // Convert to occupancy values

  gtsam::Matrix occupancy = occupancyGrid();
//  boost::filesystem::path dir(filename);

/*  if(!(boost::filesystem::exists(dir))) {
	  std::cout<<"Doesn't Exists"<<std::endl;

      if (boost::filesystem::create_directory(dir))) {
    	  std::cout << "....Successfully Created !" << std::endl;
      }
  }*/
  // Open the PGM image file
  std::ofstream image((filename + ".pgm").c_str(), std::ios::binary);

  // Write the header
  image << "P5" << std::endl; // PGM File Type
  image << boost::lexical_cast<std::string>(cols()) << " " << boost::lexical_cast<std::string>(rows()) << std::endl; // Width Height
  image << "255" << std::endl; // Max Value

  // Now write the raw data
  for(size_t row = 0; row < rows(); ++row) {
    for(size_t col = 0; col < cols(); ++col) {
      image << (unsigned char)(occupancy(row,col));
    }
  }
  image.close();



  // Open the YAML config file
  std::ofstream yaml((filename + ".yaml").c_str());
  yaml << "image: " << filename << ".pgm" << std::endl;
  yaml << "resolution: " << cell_size_ << std::endl;
  yaml << "origin: [" << origin_.x() << ", " << origin_.y() << ", " << 0.0 << " ]" << std::endl;
  yaml << "negate: 0" << std::endl;
  yaml << "occupied_thresh: 0.80" << std::endl;
  yaml << "free_thresh: 0.20" << std::endl;
  yaml.close();
}

/* ************************************************************************* */
double ProbabilityMap::LogOddsToProbability(double log_odds) {
  double odds = std::exp(log_odds);
  double probability = odds / (1.0 + odds);
  return probability;
}

/* ************************************************************************* */
double ProbabilityMap::ProbabilityToLogOdds(double probability) {
  double odds = probability / (1.0 - probability);
  double log_odds = std::log(odds);
  return log_odds;
}

/* ************************************************************************* */
} // namespace mapping


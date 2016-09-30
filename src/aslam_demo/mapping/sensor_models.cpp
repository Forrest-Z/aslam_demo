/**
 * sensor_models.cpp
 */

#include <aslam_demo/mapping/sensor_models.h>
#include <laser_geometry/laser_geometry.h>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <exception>

namespace mapping {

namespace sensor_models {

/* ************************************************************************* */
/* ************************************************************************* */
/* ************************************************************************* */
/*
Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
*
* Developed at SunPro, a Sun Microsystems, Inc. business.
* Permission to use, copy, modify, and distribute this
* software is freely granted, provided that this notice
* is preserved.
*/

//double erf(double x);
//double erfc(double x);
static const double tiny = 1e-300, half = 5.00000000000000000000e-01, /* 0x3FE00000, 0x00000000 */
one = 1.00000000000000000000e+00, /* 0x3FF00000, 0x00000000 */
two = 2.00000000000000000000e+00, /* 0x40000000, 0x00000000 */
/* c = (float)0.84506291151 */
erx = 8.45062911510467529297e-01, /* 0x3FEB0AC1, 0x60000000 */
/*
 * Coefficients for approximation to erf on [0,0.84375]
 */
efx = 1.28379167095512586316e-01, /* 0x3FC06EBA, 0x8214DB69 */
efx8 = 1.02703333676410069053e+00, /* 0x3FF06EBA, 0x8214DB69 */
pp0 = 1.28379167095512558561e-01, /* 0x3FC06EBA, 0x8214DB68 */
pp1 = -3.25042107247001499370e-01, /* 0xBFD4CD7D, 0x691CB913 */
pp2 = -2.84817495755985104766e-02, /* 0xBF9D2A51, 0xDBD7194F */
pp3 = -5.77027029648944159157e-03, /* 0xBF77A291, 0x236668E4 */
pp4 = -2.37630166566501626084e-05, /* 0xBEF8EAD6, 0x120016AC */
qq1 = 3.97917223959155352819e-01, /* 0x3FD97779, 0xCDDADC09 */
qq2 = 6.50222499887672944485e-02, /* 0x3FB0A54C, 0x5536CEBA */
qq3 = 5.08130628187576562776e-03, /* 0x3F74D022, 0xC4D36B0F */
qq4 = 1.32494738004321644526e-04, /* 0x3F215DC9, 0x221C1A10 */
qq5 = -3.96022827877536812320e-06, /* 0xBED09C43, 0x42A26120 */
/*
 * Coefficients for approximation to erf in [0.84375,1.25]
 */
pa0 = -2.36211856075265944077e-03, /* 0xBF6359B8, 0xBEF77538 */
pa1 = 4.14856118683748331666e-01, /* 0x3FDA8D00, 0xAD92B34D */
pa2 = -3.72207876035701323847e-01, /* 0xBFD7D240, 0xFBB8C3F1 */
pa3 = 3.18346619901161753674e-01, /* 0x3FD45FCA, 0x805120E4 */
pa4 = -1.10894694282396677476e-01, /* 0xBFBC6398, 0x3D3E28EC */
pa5 = 3.54783043256182359371e-02, /* 0x3FA22A36, 0x599795EB */
pa6 = -2.16637559486879084300e-03, /* 0xBF61BF38, 0x0A96073F */
qa1 = 1.06420880400844228286e-01, /* 0x3FBB3E66, 0x18EEE323 */
qa2 = 5.40397917702171048937e-01, /* 0x3FE14AF0, 0x92EB6F33 */
qa3 = 7.18286544141962662868e-02, /* 0x3FB2635C, 0xD99FE9A7 */
qa4 = 1.26171219808761642112e-01, /* 0x3FC02660, 0xE763351F */
qa5 = 1.36370839120290507362e-02, /* 0x3F8BEDC2, 0x6B51DD1C */
qa6 = 1.19844998467991074170e-02, /* 0x3F888B54, 0x5735151D */
/*
 * Coefficients for approximation to erfc in [1.25,1/0.35]
 */
ra0 = -9.86494403484714822705e-03, /* 0xBF843412, 0x600D6435 */
ra1 = -6.93858572707181764372e-01, /* 0xBFE63416, 0xE4BA7360 */
ra2 = -1.05586262253232909814e+01, /* 0xC0251E04, 0x41B0E726 */
ra3 = -6.23753324503260060396e+01, /* 0xC04F300A, 0xE4CBA38D */
ra4 = -1.62396669462573470355e+02, /* 0xC0644CB1, 0x84282266 */
ra5 = -1.84605092906711035994e+02, /* 0xC067135C, 0xEBCCABB2 */
ra6 = -8.12874355063065934246e+01, /* 0xC0545265, 0x57E4D2F2 */
ra7 = -9.81432934416914548592e+00, /* 0xC023A0EF, 0xC69AC25C */
sa1 = 1.96512716674392571292e+01, /* 0x4033A6B9, 0xBD707687 */
sa2 = 1.37657754143519042600e+02, /* 0x4061350C, 0x526AE721 */
sa3 = 4.34565877475229228821e+02, /* 0x407B290D, 0xD58A1A71 */
sa4 = 6.45387271733267880336e+02, /* 0x40842B19, 0x21EC2868 */
sa5 = 4.29008140027567833386e+02, /* 0x407AD021, 0x57700314 */
sa6 = 1.08635005541779435134e+02, /* 0x405B28A3, 0xEE48AE2C */
sa7 = 6.57024977031928170135e+00, /* 0x401A47EF, 0x8E484A93 */
sa8 = -6.04244152148580987438e-02, /* 0xBFAEEFF2, 0xEE749A62 */
/*
 * Coefficients for approximation to erfc in [1/.35,28]
 */
rb0 = -9.86494292470009928597e-03, /* 0xBF843412, 0x39E86F4A */
rb1 = -7.99283237680523006574e-01, /* 0xBFE993BA, 0x70C285DE */
rb2 = -1.77579549177547519889e+01, /* 0xC031C209, 0x555F995A */
rb3 = -1.60636384855821916062e+02, /* 0xC064145D, 0x43C5ED98 */
rb4 = -6.37566443368389627722e+02, /* 0xC083EC88, 0x1375F228 */
rb5 = -1.02509513161107724954e+03, /* 0xC0900461, 0x6A2E5992 */
rb6 = -4.83519191608651397019e+02, /* 0xC07E384E, 0x9BDC383F */
sb1 = 3.03380607434824582924e+01, /* 0x403E568B, 0x261D5190 */
sb2 = 3.25792512996573918826e+02, /* 0x40745CAE, 0x221B9F0A */
sb3 = 1.53672958608443695994e+03, /* 0x409802EB, 0x189D5118 */
sb4 = 3.19985821950859553908e+03, /* 0x40A8FFB7, 0x688C246A */
sb5 = 2.55305040643316442583e+03, /* 0x40A3F219, 0xCEDF3BE6 */
sb6 = 4.74528541206955367215e+02, /* 0x407DA874, 0xE79FE763 */
sb7 = -2.24409524465858183362e+01; /* 0xC03670E2, 0x42712D62 */

//extern double exp(double);
//extern double fabs(double);
double erf(double x) {
  int n0, hx, ix, i;
  double R, S, P, Q, s, y, z, r;
  n0 = ((*(int*) &one) >> 29) ^ 1;
  hx = *(n0 + (int*) &x);
  ix = hx & 0x7fffffff;
  if (ix >= 0x7ff00000) { /* erf(nan)=nan */
    i = ((unsigned) hx >> 31) << 1;
    return (double) (1 - i) + one / x; /* erf(+-inf)=+-1 */
  }

  if (ix < 0x3feb0000) { /* |x|<0.84375 */
    if (ix < 0x3e300000) { /* |x|<2**-28 */
      if (ix < 0x00800000)
        return 0.125 * (8.0 * x + efx8 * x); /*avoid underflow */
      return x + efx * x;
    }
    z = x * x;
    r = pp0 + z * (pp1 + z * (pp2 + z * (pp3 + z * pp4)));
    s = one + z * (qq1 + z * (qq2 + z * (qq3 + z * (qq4 + z * qq5))));
    y = r / s;
    return x + x * y;
  }
  if (ix < 0x3ff40000) { /* 0.84375 <= |x| < 1.25 */
    s = fabs(x) - one;
    P = pa0 + s * (pa1 + s * (pa2 + s * (pa3 + s * (pa4 + s * (pa5 + s * pa6)))));
    Q = one + s * (qa1 + s * (qa2 + s * (qa3 + s * (qa4 + s * (qa5 + s * qa6)))));
    if (hx >= 0)
      return erx + P / Q;
    else
      return -erx - P / Q;
  }
  if (ix >= 0x40180000) { /* inf>|x|>=6 */
    if (hx >= 0)
      return one - tiny;
    else
      return tiny - one;
  }
  x = fabs(x);
  s = one / (x * x);
  if (ix < 0x4006DB6E) { /* |x| < 1/0.35 */
    R = ra0 + s * (ra1 + s * (ra2 + s * (ra3 + s * (ra4 + s * (ra5 + s * (ra6 + s * ra7))))));
    S = one + s * (sa1 + s * (sa2 + s * (sa3 + s * (sa4 + s * (sa5 + s * (sa6 + s * (sa7 + s * sa8)))))));
  } else { /* |x| >= 1/0.35 */
    R = rb0 + s * (rb1 + s * (rb2 + s * (rb3 + s * (rb4 + s * (rb5 + s * rb6)))));
    S = one + s * (sb1 + s * (sb2 + s * (sb3 + s * (sb4 + s * (sb5 + s * (sb6 + s * sb7))))));
  }
  z = x;
  *(1 - n0 + (int*) &z) = 0;
  r = exp(-z * z - 0.5625) * exp((z - x) * (z + x) + R / S);
  if (hx >= 0)
    return one - r / x;
  else
    return r / x - one;
}

double erfc(double x) {
  int n0, hx, ix;
  double R, S, P, Q, s, y, z, r;
  n0 = ((*(int*) &one) >> 29) ^ 1;
  hx = *(n0 + (int*) &x);
  ix = hx & 0x7fffffff;
  if (ix >= 0x7ff00000) { /* erfc(nan)=nan */
    /* erfc(+-inf)=0,2 */
    return (double) (((unsigned) hx >> 31) << 1) + one / x;
  }

  if (ix < 0x3feb0000) { /* |x|<0.84375 */
    if (ix < 0x3c700000) /* |x|<2**-56 */
      return one - x;
    z = x * x;
    r = pp0 + z * (pp1 + z * (pp2 + z * (pp3 + z * pp4)));
    s = one + z * (qq1 + z * (qq2 + z * (qq3 + z * (qq4 + z * qq5))));
    y = r / s;
    if (hx < 0x3fd00000) { /* x<1/4 */
      return one - (x + x * y);
    } else {
      r = x * y;
      r += (x - half);
      return half - r;
    }
  }
  if (ix < 0x3ff40000) { /* 0.84375 <= |x| < 1.25 */
    s = fabs(x) - one;
    P = pa0 + s * (pa1 + s * (pa2 + s * (pa3 + s * (pa4 + s * (pa5 + s * pa6)))));
    Q = one + s * (qa1 + s * (qa2 + s * (qa3 + s * (qa4 + s * (qa5 + s * qa6)))));
    if (hx >= 0) {
      z = one - erx;
      return z - P / Q;
    } else {
      z = erx + P / Q;
      return one + z;
    }
  }
  if (ix < 0x403c0000) { /* |x|<28 */
    x = fabs(x);
    s = one / (x * x);
    if (ix < 0x4006DB6D) { /* |x| < 1/.35 ~ 2.857143*/
      R = ra0 + s * (ra1 + s * (ra2 + s * (ra3 + s * (ra4 + s * (ra5 + s * (ra6 + s * ra7))))));
      S = one + s * (sa1 + s * (sa2 + s * (sa3 + s * (sa4 + s * (sa5 + s * (sa6 + s * (sa7 + s * sa8)))))));
    } else { /* |x| >= 1/.35 ~ 2.857143 */
      if (hx < 0 && ix >= 0x40180000)
        return two - tiny;/* x < -6 */
      R = rb0 + s * (rb1 + s * (rb2 + s * (rb3 + s * (rb4 + s * (rb5 + s * rb6)))));
      S = one + s * (sb1 + s * (sb2 + s * (sb3 + s * (sb4 + s * (sb5 + s * (sb6 + s * sb7))))));
    }
    z = x;
    *(1 - n0 + (int*) &z) = 0;
    r = exp(-z * z - 0.5625) * exp((z - x) * (z + x) + R / S);
    if (hx > 0)
      return r / x;
    else
      return two - r / x;
  } else {
    if (hx > 0)
      return tiny * tiny;
    else
      return two - tiny;
  }
}

/* ************************************************************************* */
/* ************************************************************************* */
/* ************************************************************************* */
LaserScanModel::LaserScanModel(double range_sigma, bool use_max_range) :
  range_sigma_(range_sigma), use_max_range_(use_max_range) {
}

/* ************************************************************************* */
LaserScanModel::~LaserScanModel() {

}

/* ************************************************************************* */
void LaserScanModel::updateMap(ProbabilityMap& map, const gtsam::Point2& sensor_origin, const gtsam::Point2& laser_return) const {

  // Sensor Model (summation):
  // (1) Constant probability (Pfree) between the sensor and (range return - 3*sigma)
  // (2) Gaussian decay of 2*sigma from Pfree to 0 between (range return - 3*sigma) and infinity
  // (3) Gaussian distribution of sigma centered at range return
  // The sensor model has been implemented such that the CDF is independent of the map cell size

  // Determine the gaussian kernel size in map pixel units
  size_t kernel_size = std::ceil(3.0 * range_sigma_ / map.cellSize());

  // Compute a point 'kernel_size' pixels further than the laser return to update the gaussian tail as well
  double laser_return_distance = sensor_origin.distance(laser_return);
  gtsam::Point2 direction = sensor_origin.between(laser_return).unit();
  gtsam::Point2 end_point = sensor_origin + (laser_return_distance + kernel_size*map.cellSize())*direction;

  // Extract the points along the line from the sensor origin to the end point
  std::vector<ProbabilityMap::LineCell> line = map.line(sensor_origin, end_point);

  // Loop over the line points, calculating which sensor model segments apply
  for(size_t i = 0; i < line.size(); ++i) {

    // Compute the distance from the sensor origin to the edges of this cell
    double distance1 = sensor_origin.distance(line[i].start);
    double distance2 = sensor_origin.distance(line[i].end);
    double distance_cell = line[i].start.distance(line[i].end);

    // Integrate the likelihood update over the different model segments
    double likelihood = 0.0;

    // Check if sensor model (1) applies
    if(distance1 < (laser_return_distance-3*range_sigma_)) {
      double d_min = (sensor_origin.between(line[i].start).vector().dot(direction.vector()) < 0 ? 0.0 : distance1);
      double d_max = std::min(distance2, (laser_return_distance-3*range_sigma_));
      double clearing_probability = 1.0/(2.0*range_sigma_*sqrt(2.0*M_PI));
      likelihood -= (d_max-d_min)*clearing_probability;
    }

    // Check if sensor model (2) applies
    if(distance2 >= (laser_return_distance-3*range_sigma_)) {
      double C2 = 2.0*range_sigma_*sqrt(2.0);
      // Integrate the Gaussian distribution over the length of the cell
      double d_min = std::max(distance1, laser_return_distance-3*range_sigma_);
      double x1 = d_min - (laser_return_distance-3*range_sigma_);
      double x2 = distance2 - (laser_return_distance-3*range_sigma_);
      double cdf1 = 0.5 + 0.5*erf(x1 / C2);
      double cdf2 = 0.5 + 0.5*erf(x2 / C2);
      likelihood -= (cdf2 - cdf1);
    }

    // Check if sensor model (3) applies
    if(distance2 >= laser_return_distance-kernel_size*map.cellSize()) {
      double C3 = range_sigma_*sqrt(2.0);
      // Integrate the Gaussian distribution over length of the cell
      double x1 = distance1 - laser_return_distance;
      double x2 = distance2 - laser_return_distance;
      double cdf1 = 0.5 + 0.5*erf(x1 / C3);
      double cdf2 = 0.5 + 0.5*erf(x2 / C3);
      likelihood += (cdf2 - cdf1);
    }

    // Update the map with the probability
    map.update(line[i].row, line[i].col, 0.5 + 0.5*likelihood);
  }

}

/* ************************************************************************* */
void LaserScanModel::updateMap(ProbabilityMap& map, const sensor_msgs::LaserScan& scan, const gtsam::Pose2& world_T_base, const gtsam::Pose3& base_T_laser) const {

  // Transform the ranges into points in the laser frame
  /// @todo: The laser_geometry object automatically filters out min and max range points
  laser_geometry::LaserProjection projector;
  sensor_msgs::PointCloud cloud;
  projector.projectLaser(scan, cloud);

  // Transform the laser frame points into the world frame
  std::vector<gtsam::Point2> range_points;
  range_points.reserve(cloud.points.size());
  gtsam::Pose3 world_T_laser = gtsam::Pose3(gtsam::Rot3::Rz(world_T_base.theta()), gtsam::Point3(world_T_base.x(), world_T_base.y(), 0)) * base_T_laser;
  gtsam::Point2 sensor_origin(world_T_laser.x(), world_T_laser.y());
  for(size_t i = 0; i < cloud.points.size(); ++i) {
    gtsam::Point3 laser_P_range(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
    gtsam::Point3 world_P_range = world_T_laser * laser_P_range;
    range_points.push_back(gtsam::Point2(world_P_range.x(), world_P_range.y()));
  }

  // Update the map
  for(size_t i = 0; i < range_points.size(); ++i) {
    // Call the per-point function version
    updateMap(map, sensor_origin, range_points[i]);
  }
}




// Fancy laser sensor model that is way too slow
///* ************************************************************************* */
//void ProbabilityMap::update(const cv::Point2f& start_point, const cv::Point2f& end_point, double sigma) {
//
//  double sigma_map = sigma / cell_size_;
//
//  // Create a large image block to hold the sensor model
//  size_t kernel_radius = ceil(3.0*sigma_map);
//  size_t kernel_size = 2*kernel_radius + 1;
//  size_t distance = ceil(cv::norm(end_point - start_point));
//  size_t sensor_model_radius = distance + kernel_radius;
//  size_t sensor_model_size = 2*sensor_model_radius + 1;
//  cv::Mat sensor_model(sensor_model_size, sensor_model_size, CV_32F);
//  cv::Point sensor_model_center(sensor_model_radius, sensor_model_radius);
//  sensor_model = 0.0f;
//
//  // Add the triangular view cone of probable free space
//  {
//
//    for(size_t i = 0; i < distance; ++i) {
//      cv::Point center = sensor_model_center + cv::Point(i, 0);
//      double scale = double(i)/distance;
//      int height = ceil(scale * kernel_radius);
//      if(i > 0) {
//        for(int j = -height; j <= height; ++j) {
//          double delta = -0.5 * std::exp(-0.5 * j*j / (scale*sigma_map * scale*sigma_map));
//          sensor_model.at<float>(center+cv::Point(0,j)) = delta;
//        }
//      }
//    }
//  }
//
//  // Add a semi-circle Gaussian at the laser point as probable free space
//  {
//    // Create the full kernel
//    cv::Mat kernel_1d(kernel_size, 1, CV_32F);
//    for(size_t i = 0; i < kernel_size; ++i) {
//      double x = double(i) - kernel_radius;
//      double value = std::exp(-0.5 * x*x / (sigma_map*sigma_map));
//      kernel_1d.at<float>(i,0) = value;
//    }
//    cv::Mat kernel = -0.5 * kernel_1d * kernel_1d.t();
//
//    // Delete the left half of it
//    cv::Mat left_half(kernel, cv::Rect(0, 0, kernel_radius, kernel_size));
//    left_half = 0.0f;
//
//    // Extract a ROI around that map point
//    cv::Point center = sensor_model_center + cv::Point(distance, 0);
//    cv::Mat roi(sensor_model, cv::Rect(center.x - kernel_radius, center.y - kernel_radius, kernel_size, kernel_size));
//
//    // Add the map kernel to the any existing entries
//    roi += kernel;
//  }
//
//  // Add a Gaussian at the laser point as probable obstacle
//  {
//    // Create the full kernel
//    cv::Mat kernel_1d(kernel_size, 1, CV_32F);
//    for(size_t i = 0; i < kernel_size; ++i) {
//      double x = double(i) - kernel_radius;
//      double value = std::exp(-0.5 * x*x / (sigma_map*sigma_map));
//      kernel_1d.at<float>(i,0) = value;
//    }
//    cv::Mat kernel = +1.0 * kernel_1d * kernel_1d.t();
//
//    // Extract a ROI around that map point
//    cv::Point center = sensor_model_center + cv::Point(distance, 0);
//    cv::Mat roi(sensor_model, cv::Rect(center.x - kernel_radius, center.y - kernel_radius, kernel_size, kernel_size));
//
//    // Add the map kernel to the any existing entries
//    roi += kernel;
//  }
//
//  // Rotate sensor model to the correct angle
//  {
//    // Rotate the half-kernel to align with the laser line
//    double angle = cv::fastAtan2(end_point.y - start_point.y, end_point.x - start_point.x); // degrees
//    cv::Mat rot_mat = cv::getRotationMatrix2D(sensor_model_center, angle, 1.0);
//    cv::warpAffine(sensor_model, sensor_model, rot_mat, sensor_model.size());
//  }
//
//  // Add the sensor model to the current map data
//  {
//    // Extract a region from the map data the same size as the sensor model, centered on the starting point
//    cv::Mat roi(data_, cv::Rect(round(start_point.x) - sensor_model_radius, round(start_point.y) - sensor_model_radius, sensor_model_size, sensor_model_size));
//
//    // Add the sensor model to the existing entries
//    for (int i = 0; i < roi.rows; i++) {
//      float* roi_row = roi.ptr<float>(i);
//      float* sensor_row = sensor_model.ptr<float>(i);
//      for (int j = 0; j < roi.cols; j++) {
//        roi_row[j] += sensor_row[j];
//        if (roi_row[j] > MAX_LOG_ODDS) {
//          roi_row[j] = MAX_LOG_ODDS;
//        } else if (roi_row[j] < -MAX_LOG_ODDS) {
//          roi_row[j] = -MAX_LOG_ODDS;
//        }
//      }
//    }
//  }
//}

/* ************************************************************************* */
} // namespace sensor_models

/* ************************************************************************* */
} // namespace mapping

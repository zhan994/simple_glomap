#ifndef TYPE_H
#define TYPE_H

#include <colmap/util/logging.h>

#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace glomap {

constexpr double EPS = 1e-12;
constexpr double HALF_PI = M_PI_2;
constexpr double TWO_PI = 2 * M_PI;

struct InlierThresholdOptions {
  // Thresholds for 3D-2D matches
  double max_angle_error = 1.;          // in degree, for global positioning
  double max_reprojection_error = 1e-2; // for bundle adjustment
  double min_triangulation_angle = 1.;  // in degree, for triangulation

  // Thresholds for image_pair
  double max_epipolar_error_E = 1.;
  double max_epipolar_error_F = 4.;
  double max_epipolar_error_H = 4.;

  // Thresholds for edges
  double min_inlier_num = 30;
  double min_inlier_ratio = 0.25;
  double max_rotation_error = 10.; // in degree, for rotation averaging
};

} // namespace glomap

#endif // TYPE_H
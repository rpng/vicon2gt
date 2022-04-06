/*
 * The vicon2gt project
 * Copyright (C) 2020 Patrick Geneva
 * Copyright (C) 2020 Guoquan Huang
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef GTSAM_ROTATIONXY_H
#define GTSAM_ROTATIONXY_H

#include <Eigen/Eigen>
#include <gtsam/base/Manifold.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "utils/rpy_ops.h"

namespace gtsam {

/**
 * @brief A 3d rotation with yaw fixed at zero
 * Estimate a 2 dof rotation => Rotz(0) * Roty * Rotx
 */
class RotationXY {
private:
  double theta_x; ///< Rotation about the x-axis
  double theta_y; ///< Rotation about the y-axis

public:
  enum { dimension = 2 };

  /// Default constructor
  RotationXY() : theta_x(0), theta_y(0) {}

  /// Construct from RotationXY directly
  RotationXY(const RotationXY &obj) {
    this->theta_x = obj.theta_x;
    this->theta_y = obj.theta_y;
  }

  /// Construct from two angles
  RotationXY(const double &thetax, const double &thetay) : theta_x(thetax), theta_y(thetay) {}

  /// Return x rotation theta.
  double thetax() const { return theta_x; }

  /// Return y rotation theta.
  double thetay() const { return theta_y; }

  /// Return full 3D rotation
  Eigen::Matrix3d rot() const { return rot_y(theta_y) * rot_x(theta_x); }

  /// Retract with optional derivatives (given correction)
  RotationXY retract(const Vector2 &xi) const;

  /// Converting function from our over parameterization to the local representation (expanding about the current node's tangent space)
  Vector2 localCoordinates(const RotationXY &state) const;

  /// How this node gets printed in the ostream
  GTSAM_EXPORT
  friend std::ostream &operator<<(std::ostream &os, const RotationXY &state) {
    os << "thetax:[" << state.thetax() << "]'" << std::endl;
    os << "thetay:[" << state.thetay() << "]'" << std::endl;
    return os;
  }

  /// Print function for this node
  void print(const std::string &s = "") const { std::cout << s << *this << std::endl; }

  /// Equals function to compare this and another JPLNavState
  bool equals(const RotationXY &other, double tol = 1e-8) const {
    return gtsam::equal(theta_x, other.theta_x, tol) && gtsam::equal(theta_y, other.theta_y, tol);
  }
};

template <> struct traits<RotationXY> : internal::Manifold<RotationXY> {};

} // namespace gtsam

#endif /* GTSAM_ROTATIONXY_H */
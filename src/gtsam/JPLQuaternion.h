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
#ifndef GTSAM_JPLQUATERNION_H
#define GTSAM_JPLQUATERNION_H

#include <Eigen/Eigen>
#include <gtsam/base/Manifold.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "utils/quat_ops.h"

namespace gtsam {

/**
 * @brief JPL Quaterion
 * A quaternion that uses negative left error in JPL format.
 */
class JPLQuaternion {
private:
  Eigen::Vector4d q_GtoI; ///< Rotation from global to IMU

public:
  enum { dimension = 3 };

  /// Default constructor
  JPLQuaternion() : q_GtoI(0, 0, 0, 1) {}

  /// Construct from JPLQuaternion directly
  JPLQuaternion(const JPLQuaternion &obj) { this->q_GtoI = obj.q_GtoI; }

  /// Construct from orientation
  JPLQuaternion(const Eigen::Vector4d &q) : q_GtoI(q) {}

  /// Return rotation quaternion.
  Eigen::Vector4d q() const { return q_GtoI; }

  /// Retract with optional derivatives (given correction)
  JPLQuaternion retract(const Vector3 &xi) const;

  /// Converting function from our over parameterization to the local representation (expanding about the current node's tangent space)
  Vector3 localCoordinates(const JPLQuaternion &state) const;

  /// How this node gets printed in the ostream
  GTSAM_EXPORT
  friend std::ostream &operator<<(std::ostream &os, const JPLQuaternion &state) {
    os << "q:[" << state.q()(0) << ", " << state.q()(1) << ", " << state.q()(2) << ", " << state.q()(3) << "]'" << std::endl;
    return os;
  }

  /// Print function for this node
  void print(const std::string &s = "") const { std::cout << s << *this << std::endl; }

  /// Equals function to compare this and another JPLNavState
  bool equals(const JPLQuaternion &other, double tol = 1e-8) const { return gtsam::equal(q_GtoI, other.q_GtoI, tol); }
};

template <> struct traits<JPLQuaternion> : internal::Manifold<JPLQuaternion> {};

} // namespace gtsam

#endif /* GTSAM_JPLQUATERNION_H */
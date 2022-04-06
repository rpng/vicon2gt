/*
 * The vicon2gt project
 * Copyright (C) 2020 Patrick Geneva
 * Copyright (C) 2020 Kevin Eckenhoff
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
#ifndef GTSAM_JPLNAVSTATE_H
#define GTSAM_JPLNAVSTATE_H

#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <gtsam/base/Manifold.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <iomanip>

#include "utils/quat_ops.h"

namespace gtsam {

/// Velocity is currently typedef'd to Vector3
typedef Eigen::Vector3d Velocity3;

/// Bias for a sensor is currently typedef'd to Vector3
typedef Eigen::Vector3d Bias3;

// Define a large 15 vector as this is the size of our correction vector
typedef Eigen::Matrix<double, 15, 1> Vector15;

/**
 * @brief JPL Navigation State
 * Contains orientation, position, velocity, and biases
 */
class JPLNavState {
private:
  double m_time; ///< Timestamp that this state occurred at (seconds in IMU clock)

  Vector4 q_GtoI;   ///< Rotation from global to IMU
  Bias3 biasg;      ///< Bias of the gyroscope
  Velocity3 v_IinG; ///< Velocity of IMU in global
  Bias3 biasa;      ///< Bias of the accelerometer
  Vector3 p_IinG;   ///< Position of IMU in global

public:
  enum { dimension = 15 };

  /// Default constructor
  JPLNavState() : m_time(0), q_GtoI(0, 0, 0, 1), biasg(0, 0, 0), v_IinG(0, 0, 0), biasa(0, 0, 0), p_IinG(0, 0, 0) {}

  /// Construct from JPLNavState directly
  JPLNavState(const JPLNavState &navstate) {
    this->m_time = navstate.m_time;
    this->q_GtoI = navstate.q_GtoI;
    this->biasg = navstate.biasg;
    this->v_IinG = navstate.v_IinG;
    this->biasa = navstate.biasa;
    this->p_IinG = navstate.p_IinG;
  }

  /// Construct from orientation, position, velocity, and biases
  JPLNavState(const double &time, const Vector4 &q, const Bias3 &bgi, const Velocity3 &v, const Bias3 &bai, const Vector3 &p)
      : m_time(time), q_GtoI(q), biasg(bgi), v_IinG(v), biasa(bai), p_IinG(p) {}

  /// Return rotation quaternion.
  double time() const { return m_time; }

  /// Return rotation quaternion.
  Vector4 q() const { return q_GtoI; }

  /// Return position as Vector3
  Vector3 p() const { return p_IinG; }

  /// Return velocity as Vector3
  Vector3 v() const { return v_IinG; }

  /// Return ba as Vector3
  Vector3 ba() const { return biasa; }

  /// Return bg as Vector3
  Vector3 bg() const { return biasg; }

  /// Retract with optional derivatives (given correction, change this navstate)
  JPLNavState retract(const Vector15 &xi) const;

  /// Converting function from our overparameterization to the local representation (expanding about the current node's tangent space)
  Vector15 localCoordinates(const JPLNavState &state) const;

  /// How this node gets printed in the ostream
  GTSAM_EXPORT
  friend std::ostream &operator<<(std::ostream &os, const JPLNavState &state) {
    std::streamsize ss = os.precision();
    os << "m_time:[" << std::setprecision(15) << state.time() << std::setprecision(ss) << "]'" << std::endl;
    os << "q:[" << state.q()(0) << ", " << state.q()(1) << ", " << state.q()(2) << ", " << state.q()(3) << "]'" << std::endl;
    os << "bg:[" << state.bg()(0) << ", " << state.bg()(1) << ", " << state.bg()(2) << "]'" << std::endl;
    os << "v:[" << state.v()(0) << ", " << state.v()(1) << ", " << state.v()(2) << "]'" << std::endl;
    os << "ba:[" << state.ba()(0) << ", " << state.ba()(1) << ", " << state.ba()(2) << "]'" << std::endl;
    os << "p:[" << state.p()(0) << ", " << state.p()(1) << ", " << state.p()(2) << "]'" << std::endl;
    return os;
  }

  /// Print function for this node
  void print(const std::string &s = "") const { std::cout << s << *this << std::endl; }

  /// Equals function to compare this and another JPLNavState
  bool equals(const JPLNavState &other, double tol = 1e-8) const {
    return gtsam::equal(m_time, other.m_time, tol) && gtsam::equal(q_GtoI, other.q_GtoI, tol) && gtsam::equal(biasg, other.biasg, tol) &&
           gtsam::equal(v_IinG, other.v_IinG, tol) && gtsam::equal(biasa, other.biasa, tol) && gtsam::equal(p_IinG, other.p_IinG, tol);
  }
};

template <> struct traits<JPLNavState> : internal::Manifold<JPLNavState> {};

} // namespace gtsam

#endif /* GTSAM_JPLNAVSTATE_H */

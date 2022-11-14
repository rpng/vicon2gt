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
#ifndef GTSAM_IMUFACTORCPIv1_H
#define GTSAM_IMUFACTORCPIv1_H

#include <gtsam/base/debug.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "JPLNavState.h"
#include "RotationXY.h"
#include "utils/quat_ops.h"

using namespace gtsam;

namespace gtsam {

/// Bias for a sensor is currently typedef'd to Vector3
typedef Eigen::Vector3d Bias3;

// Define a large 15 vector as this is the size of our error state
typedef Eigen::Matrix<double, 15, 1> Vector15;

/**
 * @brief Continuous Preintegration Factor Model 1
 * Links two full JPL Navigation States with an IMU preintegrated measurement
 * NOTE: we have added rotation from vicon frame to gravity inertial frame
 * NOTE: see reference tech report for details...
 *
 * > Continuous Preintegration Theory for Graph-based Visual-Inertial Navigation
 * > Authors: Kevin Eckenhoff, Patrick Geneva, and Guoquan Huang
 * > http://udel.edu/~ghuang/papers/tr_cpi.pdf
 */
class ImuFactorCPIv1 : public NoiseModelFactor3<JPLNavState, JPLNavState, RotationXY> {
private:
  Vector3 alpha;   ///< preintegration measurement due to position
  Vector3 beta;    ///< preintegration measurement due to velocity
  Vector4 q_KtoK1; ///< preintegration measurement due to rotation

  Bias3 ba_lin; ///< original acceleration bias linerization point
  Bias3 bg_lin; ///< original gyroscope bias linearization point

  Eigen::Matrix3d J_q;     ///< jacobian of the preintegrated rotation in respect to the gyro bias correction
  Eigen::Matrix3d J_beta;  ///<  jacobian of the preintegrated velocity in respect to the gyro bias correction
  Eigen::Matrix3d J_alpha; ///<  jacobian of the preintegrated position in respect to the gyro bias correction

  Eigen::Matrix3d H_beta;  ///<  jacobian of the preintegrated velocity in respect to the accelerometer bias correction
  Eigen::Matrix3d H_alpha; ///<  jacobian of the preintegrated position in respect to the accelerometer bias correction

  double deltatime;         ///< time in seconds that this measurement is over
  double gravity_magnitude; ///< global gravity magnitude (should be the same for all measurements)

public:
  /// Construct from the two linking JPLNavStates, preingration measurement, and its covariance
  ImuFactorCPIv1(Key state_i, Key state_j, Key rotxy, Eigen::Matrix<double, 15, 15> covariance, double deltatime, double grav_m,
                 Vector3 alpha, Vector3 beta, Vector4 q_KtoK1, Bias3 ba_lin, Bias3 bg_lin, Eigen::Matrix3d J_q,
                 Eigen::Matrix3d J_beta, Eigen::Matrix3d J_alpha, Eigen::Matrix3d H_beta,
                 Eigen::Matrix3d H_alpha)
      : NoiseModelFactor3<JPLNavState, JPLNavState, RotationXY>(noiseModel::Gaussian::Covariance(covariance), state_i, state_j, rotxy) {

    // Measurement
    this->alpha = alpha;
    this->beta = beta;
    this->q_KtoK1 = q_KtoK1;
    this->ba_lin = ba_lin;
    this->bg_lin = bg_lin;
    // Precomputed Jacobians
    this->J_q = J_q;
    this->J_beta = J_beta;
    this->J_alpha = J_alpha;
    this->H_beta = H_beta;
    this->H_alpha = H_alpha;
    // Static values
    this->deltatime = deltatime;
    this->gravity_magnitude = grav_m;
  }

  /// Return alpha measurement.
  double dt() const { return deltatime; }

  /// Return alpha measurement.
  Vector3 m_alpha() const { return alpha; }

  /// Return beta measurement.
  Vector3 m_beta() const { return beta; }

  /// Return rotation delta measurement.
  Vector4 m_q() const { return q_KtoK1; }

  /// Return acceleration bias linearization point.
  Bias3 m_balin() const { return ba_lin; }

  /// Return acceleration bias linearization point.
  Bias3 m_bglin() const { return bg_lin; }

  /// Error function. Given the current states, calculate the measurement error/residual
  gtsam::Vector evaluateError(const JPLNavState &state_i, const JPLNavState &state_j, const RotationXY &gravity,
                              boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none,
                              boost::optional<Matrix &> H3 = boost::none) const;

  /// How this factor gets printed in the ostream
  GTSAM_EXPORT
  friend std::ostream &operator<<(std::ostream &os, const ImuFactorCPIv1 &factor) {
    os << "dt:[" << factor.dt() << "]'" << std::endl;
    os << "alpha:[" << factor.m_alpha()(0) << ", " << factor.m_alpha()(1) << ", " << factor.m_alpha()(2) << "]'" << std::endl;
    os << "beta:[" << factor.m_beta()(0) << ", " << factor.m_beta()(1) << ", " << factor.m_beta()(2) << "]'" << std::endl;
    os << "dq_KtoK1:[" << factor.m_q()(0) << ", " << factor.m_q()(1) << ", " << factor.m_q()(2) << ", " << factor.m_q()(3) << "]'"
       << std::endl;
    os << "ba_lin:[" << factor.m_balin()(0) << ", " << factor.m_balin()(1) << ", " << factor.m_balin()(2) << "]'" << std::endl;
    os << "bg_lin:[" << factor.m_bglin()(0) << ", " << factor.m_bglin()(1) << ", " << factor.m_bglin()(2) << "]'" << std::endl;
    return os;
  }

  /// Print function for this factor
  void print(const std::string &s, const KeyFormatter &keyFormatter = DefaultKeyFormatter) const {
    std::cout << s << "ImuFactorCPIv1(" << keyFormatter(this->key1()) << "," << keyFormatter(this->key2()) << ","
              << keyFormatter(this->key3()) << ")" << std::endl;
    std::cout << "  measured: " << std::endl << *this << std::endl;
    this->noiseModel_->print("  noise model: ");
  }

  /// Define how two factors can be equal to each other
  bool equals(const NonlinearFactor &expected, double tol = 1e-9) const {
    // Cast the object
    const auto *e = dynamic_cast<const ImuFactorCPIv1 *>(&expected);
    if (e == nullptr)
      return false;
    // Success, compare base noise values and the measurement values
    return NoiseModelFactor3<JPLNavState, JPLNavState, RotationXY>::equals(*e, tol) && gtsam::equal(deltatime, e->deltatime, tol) &&
           gtsam::equal(alpha, e->alpha, tol) && gtsam::equal(beta, e->beta, tol) && gtsam::equal(q_KtoK1, e->q_KtoK1, tol) &&
           gtsam::equal(ba_lin, e->ba_lin, tol) && gtsam::equal(bg_lin, e->bg_lin, tol) && gtsam::equal(J_q, e->J_q, tol) &&
           gtsam::equal(J_beta, e->J_beta, tol) && gtsam::equal(J_alpha, e->J_alpha, tol) && gtsam::equal(H_beta, e->H_beta, tol) &&
           gtsam::equal(H_alpha, e->H_alpha, tol);
  }
};

} // namespace gtsam

#endif /* GTSAM_IMUFACTORCPIv1_H */

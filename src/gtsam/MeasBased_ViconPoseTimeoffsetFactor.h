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
#ifndef GTSAM_VICONPOSETIMEOFFSETFACTOR_H
#define GTSAM_VICONPOSETIMEOFFSETFACTOR_H

#include <gtsam/base/debug.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "GtsamConfig.h"
#include "JPLNavState.h"
#include "JPLQuaternion.h"
#include "meas/Interpolator.h"
#include "utils/quat_ops.h"

using namespace gtsam;

namespace gtsam {

/**
 * @brief Vicon pose factor with time offset.
 * This is a measurement of the vicon "body" frame on the object in the vicon frame
 * Our states we are estimating are the JPL imu state, so we need the transform to it also
 */
class MeasBased_ViconPoseTimeoffsetFactor : public NoiseModelFactor4<JPLNavState, JPLQuaternion, Vector3, Vector1> {
private:
  std::shared_ptr<Interpolator> m_interpolator; ///< interpolator that has vicon poses in it
  std::shared_ptr<GtsamConfig> m_config;        ///< config file for if we should estimate calibration

public:
  /// Construct from the JPLNavState, calibration, and time offset
  MeasBased_ViconPoseTimeoffsetFactor(Key kstate, Key kR_BtoI, Key kp_BinI, Key kt_off, std::shared_ptr<Interpolator> interpolator,
                                      std::shared_ptr<GtsamConfig> config)
      : NoiseModelFactor4<JPLNavState, JPLQuaternion, Vector3, Vector1>(
            noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.345),
                                       noiseModel::Gaussian::Covariance(Eigen::Matrix<double, 6, 6>::Identity())),
            kstate, kR_BtoI, kp_BinI, kt_off) {
    this->m_interpolator = interpolator;
    this->m_config = config;
  }

  /// Error function. Given the current states, calculate the measurement error/residual
  gtsam::Vector evaluateError(const JPLNavState &state, const JPLQuaternion &R_BtoI, const Vector3 &p_BinI, const Vector1 &t_off,
                              boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none,
                              boost::optional<Matrix &> H3 = boost::none, boost::optional<Matrix &> H4 = boost::none) const;

  /// How this factor gets printed in the ostream
  GTSAM_EXPORT
  friend std::ostream &operator<<(std::ostream &os, const MeasBased_ViconPoseTimeoffsetFactor &factor) { return os; }

  /// Print function for this factor
  void print(const std::string &s, const KeyFormatter &keyFormatter = DefaultKeyFormatter) const {
    std::cout << s << "ViconPoseTimeoffsetFactor(" << keyFormatter(this->key1()) << "," << keyFormatter(this->key2()) << ","
              << keyFormatter(this->key3()) << "," << keyFormatter(this->key4()) << ")" << std::endl;
    this->noiseModel_->print("  noise model: ");
  }

  /// Define how two factors can be equal to each other
  bool equals(const NonlinearFactor &expected, double tol = 1e-9) const {
    // Cast the object
    const auto *e = dynamic_cast<const MeasBased_ViconPoseTimeoffsetFactor *>(&expected);
    if (e == nullptr)
      return false;
    // Success, compare base noise values and the measurement values
    return NoiseModelFactor4<JPLNavState, JPLQuaternion, Vector3, Vector1>::equals(*e, tol);
  }
};

} // namespace gtsam

#endif /* GTSAM_VICONPOSETIMEOFFSETFACTOR_H */
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
#include "MeasBased_ViconPoseTimeoffsetFactor.h"

using namespace std;
using namespace gtsam;

gtsam::Vector MeasBased_ViconPoseTimeoffsetFactor::evaluateError(const JPLNavState &state, const JPLQuaternion &q_BtoI,
                                                                 const Vector3 &p_BinI, const Vector1 &t_off, boost::optional<Matrix &> H1,
                                                                 boost::optional<Matrix &> H2, boost::optional<Matrix &> H3,
                                                                 boost::optional<Matrix &> H4) const {

  //================================================================================
  //================================================================================
  //================================================================================

  // Separate our variables from our states
  double timestamp_inI = state.time();
  Vector4 q_VtoI = state.q();
  Vector3 p_IinV = state.p();
  Vector4 q_BtoI_vec = q_BtoI.q();

  // Calculate the expected measurement values from the state
  Vector4 q_VtoB = quat_multiply(Inv(q_BtoI_vec), q_VtoI);
  Eigen::Vector3d p_BinV = p_IinV + quat_2_Rot(q_VtoI).transpose() * p_BinI;

  //================================================================================
  //================================================================================
  //================================================================================

  // Get our interpolated pose at the node timestep
  Eigen::Vector4d q_interp;
  Eigen::Vector3d p_interp;
  Eigen::Matrix<double, 6, 6> R_interp;
  Eigen::Matrix<double, 6, 1> H_toff;
  bool has_vicon = m_interpolator->get_pose_with_jacobian(timestamp_inI - t_off(0), q_interp, p_interp, R_interp, H_toff);

  // Find the sqrt inverse to whittening
  // This is because our measurement noise can change every iteration based on interpolation
  Eigen::Matrix<double, 6, 6> sqrt_inv_interp = R_interp.llt().matrixL();
  sqrt_inv_interp = sqrt_inv_interp.colPivHouseholderQr().solve(Eigen::Matrix<double, 6, 6>::Identity());

  // Our error vector [delta = (orientation, position)]
  Vector6 error;
  Vector4 q_r = quat_multiply(q_VtoB, Inv(q_interp));

  // Error in our our state in respect to the measurement
  error.block(0, 0, 3, 1) = 2 * q_r.block(0, 0, 3, 1);
  error.block(3, 0, 3, 1) = p_BinV - p_interp;
  error = sqrt_inv_interp * error;

  //================================================================================
  //================================================================================
  //================================================================================

  // Compute the Jacobian in respect to the first JPLNavState if needed
  if (H1) {
    Eigen::Matrix<double, 6, 15> H = Eigen::Matrix<double, 6, 15>::Zero();
    if (has_vicon) {
      H.block(0, 0, 3, 3) = quat_2_Rot(Inv(q_BtoI_vec));
      H.block(3, 0, 3, 3) = -quat_2_Rot(Inv(q_VtoI)) * skew_x(p_BinI);
      H.block(3, 12, 3, 3) = Eigen::Matrix3d::Identity();
      H = sqrt_inv_interp * H;
    }
    *H1 = *OptionalJacobian<6, 15>(H);
    //*H1 = numericalDerivative41<Vector,JPLNavState,JPLQuaternion,Vector3,Vector1>(
    //        boost::bind(&ViconPoseTimeoffsetFactor::evaluateError, this,
    //                    _1, _2, _3, _4,
    //                    boost::none, boost::none, boost::none, boost::none),
    //        state, R_BtoI, p_BinI, t_off);
  }

  // Compute the Jacobian in respect orientation extrinics between BODY and IMU frames
  // NOTE: gtsam uses the right exponential expansion of the rotation error
  // NOTE: (I-skew(theta1))*R_VtoB = (R_BtoI*(I+skew(theta2)))^T*R_VtoI
  // NOTE: thus if we used the Rot3 we would have this Jacobian instead:
  // NOTE: H.block(0, 0, 3, 3).setIdentity();
  if (H2) {
    Eigen::Matrix<double, 6, 3> H = Eigen::Matrix<double, 6, 3>::Zero();
    if (has_vicon && m_config->estimate_vicon_imu_ori) {
      H.block(0, 0, 3, 3) = -quat_2_Rot(Inv(q_BtoI_vec));
      H = sqrt_inv_interp * H;
    }
    *H2 = *OptionalJacobian<6, 3>(H);
    //*H2 = numericalDerivative42<Vector,JPLNavState,JPLQuaternion,Vector3,Vector1>(
    //        boost::bind(&ViconPoseTimeoffsetFactor::evaluateError, this,
    //                    _1, _2, _3, _4,
    //                    boost::none, boost::none, boost::none, boost::none),
    //        state, R_BtoI, p_BinI, t_off);
  }

  // Compute the Jacobian in respect position extrinsics between BODY and IMU frames
  if (H3) {
    Eigen::Matrix<double, 6, 3> H = Eigen::Matrix<double, 6, 3>::Zero();
    if (has_vicon && m_config->estimate_vicon_imu_pos) {
      H.block(3, 0, 3, 3) = quat_2_Rot(Inv(q_VtoI));
      H = sqrt_inv_interp * H;
    }
    *H3 = *OptionalJacobian<6, 3>(H);
    //*H3 = numericalDerivative43<Vector,JPLNavState,JPLQuaternion,Vector3,Vector1>(
    //        boost::bind(&ViconPoseTimeoffsetFactor::evaluateError, this,
    //                    _1, _2, _3, _4,
    //                    boost::none, boost::none, boost::none, boost::none),
    //        state, R_BtoI, p_BinI, t_off);
  }

  // Compute the Jacobian in respect time offset
  // NOTE: the measurement itself is a function of time offset
  // NOTE: thus we here bring it over to the "other side" of the measurement equation and thus make it negative
  if (H4) {
    Eigen::Matrix<double, 6, 1> H = Eigen::Matrix<double, 6, 1>::Zero();
    if (has_vicon && m_config->estimate_vicon_imu_toff) {
      H = -H_toff;
      H = sqrt_inv_interp * H;
    }
    *H4 = *OptionalJacobian<6, 1>(H);
    //*H4 = numericalDerivative44<Vector,JPLNavState,JPLQuaternion,Vector3,Vector1>(
    //        boost::bind(&ViconPoseTimeoffsetFactor::evaluateError, this,
    //                    _1, _2, _3, _4,
    //                    boost::none, boost::none, boost::none, boost::none),
    //        state, R_BtoI, p_BinI, t_off, 1e-5);
  }

  // Finally return our error vector!
  return error;
}

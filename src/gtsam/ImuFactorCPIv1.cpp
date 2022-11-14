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
#include "ImuFactorCPIv1.h"

using namespace std;
using namespace gtsam;

gtsam::Vector ImuFactorCPIv1::evaluateError(const JPLNavState &state_i, const JPLNavState &state_j, const RotationXY &rotxy,
                                            boost::optional<Matrix &> H1, boost::optional<Matrix &> H2,
                                            boost::optional<Matrix &> H3) const {

  // Separate our variables from our states
  Vector4 q_GtoK = state_i.q();
  Vector4 q_GtoK1 = state_j.q();
  Bias3 bg_K = state_i.bg();
  Bias3 bg_K1 = state_j.bg();
  Velocity3 v_KinG = state_i.v();
  Velocity3 v_K1inG = state_j.v();
  Bias3 ba_K = state_i.ba();
  Bias3 ba_K1 = state_j.ba();
  Vector3 p_KinG = state_i.p();
  Vector3 p_K1inG = state_j.p();

  // Estimated gravity in the global frame
  // We do this so our global frame doesn't have to be gravity aligned
  // Global frame in this this file is the vicon frame
  Vector3 ez = {0, 0, 1};
  Vector3 grav_inG = rotxy.rot() * gravity_magnitude * ez;

  //================================================================================
  //================================================================================
  //================================================================================

  // Calculate effect of bias on the orientation
  Eigen::Matrix3d ExpB = exp_so3(-J_q.block(0, 0, 3, 3) * (bg_K - bg_lin));
  Vector4 q_b = rot_2_quat(ExpB);

  // Quaternions used in Jacobian calculations
  Vector4 q_n = quat_multiply(q_GtoK1, Inv(q_GtoK));
  Vector4 q_rminus = quat_multiply(q_n, Inv(q_KtoK1));
  Vector4 q_r = quat_multiply(q_rminus, q_b);
  Vector4 q_m = quat_multiply(Inv(q_b), q_KtoK1);

  // Calculate the corrected quaternion (q_GtoK*q_GtoK1^-1*q_meas^-1*q_b)
  // Vector4 q_r = quat_multiply(quat_multiply(q_GtoK1,Inv(q_GtoK)),quat_multiply(Inv(q_KtoK1),q_b));

  // Calculate the expected measurement values from the state
  Vector3 alphahat = quat_2_Rot(q_GtoK) * (p_K1inG - p_KinG - v_KinG * deltatime + 0.5 * grav_inG * std::pow(deltatime, 2));
  alphahat = alphahat - J_alpha * (bg_K - bg_lin) - H_alpha * (ba_K - ba_lin);
  Vector3 betahat = quat_2_Rot(q_GtoK) * (v_K1inG - v_KinG + grav_inG * deltatime);
  betahat = betahat - J_beta * (bg_K - bg_lin) - H_beta * (ba_K - ba_lin);

  //================================================================================
  //================================================================================
  //================================================================================

  // Our error vector [delta = (orientation, bg, v, ba, pos)]
  Vector15 error;

  // Error in our our state in respect to the measurement
  error.block(0, 0, 3, 1) = 2 * q_r.block(0, 0, 3, 1);
  error.block(3, 0, 3, 1) = bg_K1 - bg_K;
  error.block(6, 0, 3, 1) = betahat - beta;
  error.block(9, 0, 3, 1) = ba_K1 - ba_K;
  error.block(12, 0, 3, 1) = alphahat - alpha;

  //================================================================================
  //================================================================================
  //================================================================================

  // Compute the Jacobian in respect to the first JPLNavState if needed
  if (H1) {

    // Create our five jacobians of the measurement in respect to the state
    Eigen::Matrix<double, 15, 3> H_theta = Eigen::Matrix<double, 15, 3>::Zero();
    Eigen::Matrix<double, 15, 3> H_biasg = Eigen::Matrix<double, 15, 3>::Zero();
    Eigen::Matrix<double, 15, 3> H_velocity = Eigen::Matrix<double, 15, 3>::Zero();
    Eigen::Matrix<double, 15, 3> H_biasa = Eigen::Matrix<double, 15, 3>::Zero();
    Eigen::Matrix<double, 15, 3> H_position = Eigen::Matrix<double, 15, 3>::Zero();

    //===========================================================
    // Derivative of q_meas in respect to state theta (t=K)
    H_theta.block(0, 0, 3, 3) = -((q_n(3, 0) * Eigen::MatrixXd::Identity(3, 3) - skew_x(q_n.block(0, 0, 3, 1))) *
                                      (q_m(3, 0) * Eigen::MatrixXd::Identity(3, 3) - skew_x(q_m.block(0, 0, 3, 1))) +
                                  q_n.block(0, 0, 3, 1) * (q_m.block(0, 0, 3, 1)).transpose());
    // Derivative of beta in respect to state theta (t=K)
    H_theta.block(6, 0, 3, 3) = skew_x(quat_2_Rot(q_GtoK) * (v_K1inG - v_KinG + grav_inG * deltatime));
    // Derivative of alpha in respect to state theta (t=K)
    H_theta.block(12, 0, 3, 3) =
        skew_x(quat_2_Rot(q_GtoK) * (p_K1inG - p_KinG - v_KinG * deltatime + 0.5 * grav_inG * std::pow(deltatime, 2)));

    //===========================================================
    // Derivative of q_meas in respect to state biasg (t=K)
    H_biasg.block(0, 0, 3, 3) =
        (q_rminus(3, 0) * Eigen::MatrixXd::Identity(3, 3) - skew_x(q_rminus.block(0, 0, 3, 1))) * J_q.block(0, 0, 3, 3);
    // Derivative of biasg in respect to state biasg (t=K)
    H_biasg.block(3, 0, 3, 3) = -Eigen::MatrixXd::Identity(3, 3);
    // Derivative of beta in respect to state biasg (t=K)
    H_biasg.block(6, 0, 3, 3) = -J_beta;
    // Derivative of alpha in respect to state biasg (t=K)
    H_biasg.block(12, 0, 3, 3) = -J_alpha;

    //===========================================================
    // Derivative of beta in respect to state velocity (t=K)
    H_velocity.block(6, 0, 3, 3) = -quat_2_Rot(q_GtoK);
    // Derivative of alpha in respect to state velocity (t=K)
    H_velocity.block(12, 0, 3, 3) = -deltatime * quat_2_Rot(q_GtoK);

    //===========================================================
    // Derivative of beta in respect to state biasa (t=K)
    H_biasa.block(6, 0, 3, 3) = -H_beta;
    // Derivative of biasa in respect to state biasa (t=K)
    H_biasa.block(9, 0, 3, 3) = -Eigen::MatrixXd::Identity(3, 3);
    // Derivative of alpha in respect to state biasa (t=K)
    H_biasa.block(12, 0, 3, 3) = -H_alpha;

    //===========================================================
    // Derivative of alpha in respect to state position (t=K)
    H_position.block(12, 0, 3, 3) = -quat_2_Rot(q_GtoK);

    //===========================================================
    // Reconstruct the whole Jacobian from the different columns
    Eigen::Matrix<double, 15, 15> Hi;
    Hi.block(0, 0, 15, 3) = H_theta;
    Hi.block(0, 3, 15, 3) = H_biasg;
    Hi.block(0, 6, 15, 3) = H_velocity;
    Hi.block(0, 9, 15, 3) = H_biasa;
    Hi.block(0, 12, 15, 3) = H_position;
    *H1 = *OptionalJacobian<15, 15>(Hi);
  }

  // Compute the Jacobian in respect to the second JPLNavState if needed
  if (H2) {

    // Create our five jacobians of the measurement in respect to the state
    Eigen::Matrix<double, 15, 3> H_theta = Eigen::Matrix<double, 15, 3>::Zero();
    Eigen::Matrix<double, 15, 3> H_biasg = Eigen::Matrix<double, 15, 3>::Zero();
    Eigen::Matrix<double, 15, 3> H_velocity = Eigen::Matrix<double, 15, 3>::Zero();
    Eigen::Matrix<double, 15, 3> H_biasa = Eigen::Matrix<double, 15, 3>::Zero();
    Eigen::Matrix<double, 15, 3> H_position = Eigen::Matrix<double, 15, 3>::Zero();

    //===========================================================
    // Derivative of q_meas in respect to state theta (t=K+1)
    H_theta.block(0, 0, 3, 3) = q_r(3, 0) * Eigen::MatrixXd::Identity(3, 3) + skew_x(q_r.block(0, 0, 3, 1));

    //===========================================================
    // Derivative of biasg in respect to state biasg (t=K+1)
    H_biasg.block(3, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);

    //===========================================================
    // Derivative of beta in respect to state velocity (t=K+1)
    H_velocity.block(6, 0, 3, 3) = quat_2_Rot(q_GtoK);

    //===========================================================
    // Derivative of biasa in respect to state biasa (t=K+1)
    H_biasa.block(9, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);

    //===========================================================
    // Derivative of alpha in respect to state position (t=K+1)
    H_position.block(12, 0, 3, 3) = quat_2_Rot(q_GtoK);

    //===========================================================
    // Reconstruct the whole Jacobian
    Eigen::Matrix<double, 15, 15> Hj;
    Hj.block(0, 0, 15, 3) = H_theta;
    Hj.block(0, 3, 15, 3) = H_biasg;
    Hj.block(0, 6, 15, 3) = H_velocity;
    Hj.block(0, 9, 15, 3) = H_biasa;
    Hj.block(0, 12, 15, 3) = H_position;
    *H2 = *OptionalJacobian<15, 15>(Hj);
  }

  // Jacobian in respect to the global gravity rotation into vicon frame
  if (H3) {
    // Our jacobian
    Eigen::Matrix<double, 3, 2> H_thetaxy = Eigen::Matrix<double, 3, 2>::Zero();
    H_thetaxy.block(0, 0, 3, 1) << -std::sin(rotxy.thetay()) * std::sin(rotxy.thetax()), -std::cos(rotxy.thetax()),
        -std::cos(rotxy.thetay()) * std::sin(rotxy.thetax());
    H_thetaxy.block(0, 1, 3, 1) << std::cos(rotxy.thetay()) * std::cos(rotxy.thetax()), 0.0,
        -std::sin(rotxy.thetay()) * std::cos(rotxy.thetax());
    // Derivative of beta, alpha, in respect to our two rotation angles
    Eigen::Matrix<double, 15, 2> Hg = Eigen::Matrix<double, 15, 2>::Zero();
    Hg.block(6, 0, 3, 2) = quat_2_Rot(q_GtoK) * deltatime * gravity_magnitude * H_thetaxy;
    Hg.block(12, 0, 3, 2) = 0.5 * quat_2_Rot(q_GtoK) * std::pow(deltatime, 2) * gravity_magnitude * H_thetaxy;
    // Reconstruct the whole Jacobian
    *H3 = *OptionalJacobian<15, 2>(Hg);
  }

  // Debug printing of error and Jacobians
  // if(H1) cout << endl << "ImuFactorCPIv1 H1" << endl << *H1 << endl << endl;
  // if(H2) cout << endl << "ImuFactorCPIv1 H2" << endl << *H2 << endl << endl;
  // if(H3) cout << endl << "ImuFactorCPIv1 H3" << endl << *H3 << endl << endl;
  // KeyFormatter keyFormatter = DefaultKeyFormatter;
  // cout << endl << "ImuFactorCPIv1 (" << keyFormatter(this->key1()) << "," << keyFormatter(this->key2()) << "," <<
  // keyFormatter(this->key3()) << ")" << endl << error.transpose() << endl;

  // Finally return our error vector!
  return error;
}

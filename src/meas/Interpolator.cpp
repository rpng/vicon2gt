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
#include "Interpolator.h"

void Interpolator::feed_pose(double timestamp, Eigen::Vector4d q, Eigen::Vector3d p, Eigen::Matrix3d R_q, Eigen::Matrix3d R_p) {

  // Create our imu data object
  POSEDATA data;
  data.timestamp = timestamp;
  data.has_odom = false;
  data.q = q;
  data.p = p;
  data.R_q = R_q;
  data.R_p = R_p;

  // Append it to our vector
  pose_data.insert(data);

  // Update our times
  time_min = std::min(time_min, timestamp);
  time_max = std::max(time_max, timestamp);
}

void Interpolator::feed_odom(double timestamp, Eigen::Vector4d q, Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d w,
                             Eigen::Matrix3d R_q, Eigen::Matrix3d R_p, Eigen::Matrix3d R_v, Eigen::Matrix3d R_w) {

  // Create our imu data object
  POSEDATA data;
  data.timestamp = timestamp;
  data.has_odom = true;
  data.q = q;
  data.p = p;
  data.v = v;
  data.w = w;
  data.R_q = R_q;
  data.R_p = R_p;
  data.R_v = R_v;
  data.R_w = R_w;

  // Append it to our vector
  pose_data.insert(data);
}

bool Interpolator::get_pose(double timestamp, Eigen::Vector4d &q, Eigen::Vector3d &p, Eigen::Matrix<double, 6, 6> &R) {

  // Find our bounds for the desired timestamp
  POSEDATA pose_to_find;
  pose_to_find.timestamp = timestamp;
  auto bounds = pose_data.equal_range(pose_to_find);

  // Best we can do at the beginning is just the first vicon pose
  if (bounds.first == pose_data.begin()) {
    // our pose
    // POSEDATA poseEXACT = *bounds.first;
    // mean values
    q << 0, 0, 0, 1; //= poseEXACT.q;
    p << 0, 0, 0;    //= poseEXACT.p;
    // meas covariance
    R.setZero();
    // R.block(0,0,3,3) = poseEXACT.R_q;
    // R.block(3,3,3,3) = poseEXACT.R_p;
    return false;
  }

  // Return false if we do not have any bounding pose for this measurement (shouldn't happen)
  if (bounds.first == pose_data.end() || bounds.second == pose_data.end()) {
    // our pose
    // POSEDATA poseEXACT = *(--bounds.first);
    // mean values
    q << 0, 0, 0, 1; //= poseEXACT.q;
    p << 0, 0, 0;    //= poseEXACT.p;
    // meas covariance
    R.setZero();
    // R.block(0,0,3,3) = poseEXACT.R_q;
    // R.block(3,3,3,3) = poseEXACT.R_p;
    // ROS_ERROR("[INTER]: UNABLE TO FIND BOUNDING POSES, %d, %d",bounds.first==pose_data.end(),bounds.second==pose_data.end());
    // ROS_ERROR("[INTER]: tmeas = %.9f | time0 = %.9f | time1 = %.9f", timestamp, time0, time1);
    return false;
  }
  bounds.first--;

  // If we found an exact one, just return that
  if (bounds.first->timestamp == bounds.second->timestamp) {
    // our pose
    POSEDATA poseEXACT = *bounds.first;
    // mean values
    q = poseEXACT.q;
    p = poseEXACT.p;
    // meas covariance
    R.setZero();
    R.block(0, 0, 3, 3) = poseEXACT.R_q;
    R.block(3, 3, 3, 3) = poseEXACT.R_p;
    return true;
  }

  // Else set our bounds as the bounds our binary search found
  POSEDATA pose0 = *bounds.first;
  POSEDATA pose1 = *bounds.second;

  // Return failure if the poses are too far away
  // NOTE: Right now we just say that the poses need to be at least 5 seconds away
  // NOTE: This might cause failure if low frequency vicon rates, but 5 second is pretty slow...
  // NOTE: The pose estimate is only really used for initial guess, we will be stricter on accepting info for the factor...
  double thresh_sec = 5.0;
  if (std::abs(timestamp - pose0.timestamp) > thresh_sec || std::abs(timestamp - pose1.timestamp) > thresh_sec) {
    // ROS_ERROR("[INTER]: UNABLE TO FIND BOUNDING POSES, %d, %d", bounds.first == pose_data.end(), bounds.second == pose_data.end());
    // ROS_ERROR("[INTER]: tmeas = %.9f | time0 = %.9f | time1 = %.9f", timestamp, pose0.timestamp, pose1.timestamp);
    return false;
  }

  // Our lamda time-distance fraction
  double lambda = (timestamp - pose0.timestamp) / (pose1.timestamp - pose0.timestamp);

  // Bounding SO(3) orientations
  Eigen::Matrix3d R_Gto0 = quat_2_Rot(pose0.q);
  Eigen::Matrix3d R_Gto1 = quat_2_Rot(pose1.q);

  // Now perform the interpolation
  Eigen::Matrix3d R_0to1 = R_Gto1 * R_Gto0.transpose();
  Eigen::Matrix3d R_0toi = exp_so3(lambda * log_so3(R_0to1));
  Eigen::Matrix3d R_interp = R_0toi * R_Gto0;
  Eigen::Vector3d p_interp = (1 - lambda) * pose0.p + lambda * pose1.p;

  // Calculate intermediate values for cov propagation equations
  // Equation (8)-(10) of Geneva2018ICRA async measurement paper
  Eigen::Matrix3d eye33 = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d JR_r0i = Jr_so3(lambda * log_so3(R_0to1));
  Eigen::Matrix3d JRinv_r01 = Jr_so3(log_so3(R_0to1)).inverse();
  JRinv_r01 = JRinv_r01.inverse().eval();
  Eigen::Matrix3d JRneg_r0i = Jr_so3(-lambda * log_so3(R_0to1.transpose()));
  Eigen::Matrix3d JRneginv_r01 = Jr_so3(log_so3(R_0to1.transpose())).inverse();
  JRneginv_r01 = JRneginv_r01.inverse().eval();

  // Covariance propagation Jacobian
  // Equation (7) of Geneva2018ICRA async measurement paper
  Eigen::Matrix<double, 6, 12> Hu = Eigen::Matrix<double, 6, 12>::Zero();
  Hu.block(0, 0, 3, 3) = -R_0toi * (JR_r0i * lambda * JRinv_r01 - eye33);
  Hu.block(0, 6, 3, 3) = R_0toi * (JRneg_r0i * lambda * JRinv_r01);
  Hu.block(3, 6, 3, 3) = (1 - lambda) * eye33;
  Hu.block(3, 9, 3, 3) = lambda * eye33;

  // Finally propagate the covariance!
  Eigen::Matrix<double, 12, 12> R_12 = Eigen::Matrix<double, 12, 12>::Zero();
  R_12.block(0, 0, 3, 3) = pose0.R_q;
  R_12.block(3, 3, 3, 3) = pose0.R_p;
  R_12.block(6, 6, 3, 3) = pose1.R_q;
  R_12.block(9, 9, 3, 3) = pose1.R_p;
  R = Hu * R_12 * Hu.transpose();

  // Done
  q = rot_2_quat(R_interp);
  p = p_interp;
  return true;
}

bool Interpolator::get_pose_with_jacobian(double timestamp, Eigen::Vector4d &q, Eigen::Vector3d &p, Eigen::Matrix<double, 6, 6> &R,
                                          Eigen::Matrix<double, 6, 1> &H_toff) {

  // Find our bounds for the desired timestamp
  POSEDATA pose_to_find;
  pose_to_find.timestamp = timestamp;
  auto bounds = pose_data.equal_range(pose_to_find);

  // Best we can do at the beginning is just the first vicon pose
  if (bounds.first == pose_data.begin()) {
    // our pose
    // POSEDATA poseEXACT = *bounds.first;
    // mean values
    q << 0, 0, 0, 1; //= poseEXACT.q;
    p << 0, 0, 0;    //= poseEXACT.p;
    // meas covariance
    R.setZero();
    // R.block(0,0,3,3) = poseEXACT.R_q;
    // R.block(3,3,3,3) = poseEXACT.R_p;
    return false;
  }

  // Return false if we do not have any bounding pose for this measurement (shouldn't happen)
  if (bounds.first == pose_data.end() || bounds.second == pose_data.end()) {
    // our pose
    // POSEDATA poseEXACT = *(--bounds.first);
    // mean values
    q << 0, 0, 0, 1; //= poseEXACT.q;
    p << 0, 0, 0;    //= poseEXACT.p;
    // meas covariance
    R.setZero();
    // R.block(0,0,3,3) = poseEXACT.R_q;
    // R.block(3,3,3,3) = poseEXACT.R_p;
    // ROS_ERROR("[INTER]: UNABLE TO FIND BOUNDING POSES, %d, %d",bounds.first==pose_data.end(),bounds.second==pose_data.end());
    // ROS_ERROR("[INTER]: tmeas = %.9f | time0 = %.9f | time1 = %.9f", timestamp, time0, time1);
    return false;
  }
  bounds.first--;

  // If we found an exact one, just return that
  if (bounds.first->timestamp == bounds.second->timestamp) {
    // our pose
    POSEDATA poseEXACT = *bounds.first;
    // mean values
    q = poseEXACT.q;
    p = poseEXACT.p;
    // meas covariance
    R.setZero();
    R.block(0, 0, 3, 3) = poseEXACT.R_q;
    R.block(3, 3, 3, 3) = poseEXACT.R_p;
    return true;
  }

  // Else set our bounds as the bounds our binary search found
  POSEDATA pose0 = *bounds.first;
  POSEDATA pose1 = *bounds.second;

  // Return failure if the poses are too far away
  // NOTE: this is more strict since we don't want to use vicon measurements that are too far away (5hz)
  // NOTE: the factor will not add any information if we return false, so this is ok to be stricter here...
  double thresh_sec = 0.1;
  if (std::abs(timestamp - pose0.timestamp) > thresh_sec || std::abs(timestamp - pose1.timestamp) > thresh_sec) {
    // ROS_ERROR("[INTER]: UNABLE TO FIND BOUNDING POSES, %d, %d", bounds.first == pose_data.end(), bounds.second == pose_data.end());
    // ROS_ERROR("[INTER]: tmeas = %.9f | time0 = %.9f | time1 = %.9f", timestamp, pose0.timestamp, pose1.timestamp);
    return false;
  }

  // Our lamda time-distance fraction
  double lambda = (timestamp - pose0.timestamp) / (pose1.timestamp - pose0.timestamp);

  // Bounding SO(3) orientations
  Eigen::Matrix3d R_Gto0 = quat_2_Rot(pose0.q);
  Eigen::Matrix3d R_Gto1 = quat_2_Rot(pose1.q);

  // Now perform the interpolation
  Eigen::Matrix3d R_0to1 = R_Gto1 * R_Gto0.transpose();
  Eigen::Matrix3d R_0toi = exp_so3(lambda * log_so3(R_0to1));
  Eigen::Matrix3d R_interp = R_0toi * R_Gto0;
  Eigen::Vector3d p_interp = (1 - lambda) * pose0.p + lambda * pose1.p;

  // Calculate intermediate values for cov propagation equations
  // Equation (8)-(10) of Geneva2018ICRA async measurement paper
  Eigen::Matrix3d eye33 = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d JR_r0i = Jr_so3(lambda * log_so3(R_0to1));
  Eigen::Matrix3d JRinv_r01 = Jr_so3(log_so3(R_0to1)).inverse();
  JRinv_r01 = JRinv_r01.inverse().eval();
  Eigen::Matrix3d JRneg_r0i = Jr_so3(-lambda * log_so3(R_0to1.transpose()));
  Eigen::Matrix3d JRneginv_r01 = Jr_so3(log_so3(R_0to1.transpose())).inverse();
  JRneginv_r01 = JRneginv_r01.inverse().eval();

  // Covariance propagation Jacobian
  // Equation (7) of Geneva2018ICRA async measurement paper
  Eigen::Matrix<double, 6, 12> Hu = Eigen::Matrix<double, 6, 12>::Zero();
  Hu.block(0, 0, 3, 3) = -R_0toi * (JR_r0i * lambda * JRinv_r01 - eye33);
  Hu.block(0, 6, 3, 3) = R_0toi * (JRneg_r0i * lambda * JRinv_r01);
  Hu.block(3, 6, 3, 3) = (1 - lambda) * eye33;
  Hu.block(3, 9, 3, 3) = lambda * eye33;

  // Finally propagate the covariance!
  Eigen::Matrix<double, 12, 12> R_12 = Eigen::Matrix<double, 12, 12>::Zero();
  R_12.block(0, 0, 3, 3) = pose0.R_q;
  R_12.block(3, 3, 3, 3) = pose0.R_p;
  R_12.block(6, 6, 3, 3) = pose1.R_q;
  R_12.block(9, 9, 3, 3) = pose1.R_p;
  R = Hu * R_12 * Hu.transpose();

  // Jacobian in respect to our time offset
  double H_lambda2toff = -1.0 / (pose1.timestamp - pose0.timestamp);
  H_toff.setZero();
  H_toff.block(0, 0, 3, 1) = -R_0toi * JR_r0i * log_so3(R_0to1) * H_lambda2toff;
  H_toff.block(3, 0, 3, 1) = (pose1.p - pose0.p) * H_lambda2toff;

  // Done
  q = rot_2_quat(R_interp);
  p = p_interp;
  return true;
}

bool Interpolator::get_bounds(double timestamp, double &time0, Eigen::Vector4d &q0, Eigen::Vector3d &p0, Eigen::Matrix<double, 6, 6> &R0,
                              double &time1, Eigen::Vector4d &q1, Eigen::Vector3d &p1, Eigen::Matrix<double, 6, 6> &R1) {

  // Find our bounds for the desired timestamp
  POSEDATA pose_to_find;
  pose_to_find.timestamp = timestamp;
  auto bounds = pose_data.equal_range(pose_to_find);

  // Return false if we do not have any bounding pose for this measurement (shouldn't happen)
  if (bounds.first == pose_data.begin() || bounds.first == pose_data.end() || bounds.second == pose_data.end()) {
    // ROS_ERROR("[INTERPOLATOR]: UNABLE TO FIND BOUNDING POSES, %d, %d",bounds.first==pose_data.end(),bounds.second==pose_data.end());
    // ROS_ERROR("[INTERPOLATOR]: tmeas = %.9f | time0 = %.9f | time1 = %.9f", timestamp, time0, time1);
    return false;
  }
  bounds.first--;

  // Else set our bounds as the bounds our binary search found
  POSEDATA pose0 = *bounds.first;
  POSEDATA pose1 = *bounds.second;

  // Pose 0
  time0 = pose0.timestamp;
  q0 = pose0.q;
  p0 = pose0.p;
  R0.setZero();
  R0.block(0, 0, 3, 3) = pose0.R_q;
  R0.block(3, 3, 3, 3) = pose0.R_p;

  // Pose 1
  time1 = pose1.timestamp;
  q1 = pose1.q;
  p1 = pose1.p;
  R1.setZero();
  R1.block(0, 0, 3, 3) = pose1.R_q;
  R1.block(3, 3, 3, 3) = pose1.R_p;

  return true;
}

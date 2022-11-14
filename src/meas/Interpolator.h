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
#ifndef INTERPOLATOR_H
#define INTERPOLATOR_H

#include <Eigen/Eigen>
#include <algorithm>
#include <ros/ros.h>
#include <vector>

#include "cpi/CpiV1.h"
#include "utils/quat_ops.h"

struct POSEDATA {
  double timestamp = -1;
  bool has_odom = false;
  // ori, pos, vel, ang
  Eigen::Vector4d q;
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d w;
  // noise covariances
  Eigen::Matrix3d R_q;
  Eigen::Matrix3d R_p;
  Eigen::Matrix3d R_v;
  Eigen::Matrix3d R_w;
  // Comparison operator
  bool operator<(const POSEDATA &s) const { return timestamp < s.timestamp; }
  bool operator>(const POSEDATA &s) const { return timestamp > s.timestamp; }
};

class Interpolator {

public:
  // Default constuctor
  Interpolator() {}

  /// Our feed function for POSE measurements
  void feed_pose(double timestamp, Eigen::Vector4d q, Eigen::Vector3d p, Eigen::Matrix3d R_q, Eigen::Matrix3d R_p);

  /// Our feed function for ODOM measurements
  void feed_odom(double timestamp, Eigen::Vector4d q, Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d w, Eigen::Matrix3d R_q,
                 Eigen::Matrix3d R_p, Eigen::Matrix3d R_v, Eigen::Matrix3d R_w);

  /// Given a timestamp, this will get the pose at that time
  /// If we don't have that pose in our vector, we will perform interpolation to get it
  bool get_pose(double timestamp, Eigen::Vector4d &q, Eigen::Vector3d &p, Eigen::Matrix<double, 6, 6> &R);

  /// Given a timestamp, this will get the pose at that time
  /// If we don't have that pose in our vector, we will perform interpolation to get it
  bool get_pose_with_jacobian(double timestamp, Eigen::Vector4d &q, Eigen::Vector3d &p, Eigen::Matrix<double, 6, 6> &R,
                              Eigen::Matrix<double, 6, 1> &H_toff);

  /// Given a timestamp, this will find the bounding poses for them
  bool get_bounds(double timestamp, double &time0, Eigen::Vector4d &q0, Eigen::Vector3d &p0, Eigen::Matrix<double, 6, 6> &R0, double &time1,
                  Eigen::Vector4d &q1, Eigen::Vector3d &p1, Eigen::Matrix<double, 6, 6> &R1);

  /// Get all raw poses (used only for viz)
  std::set<POSEDATA, std::less<POSEDATA>> get_raw_poses() { return pose_data; }

  /// Oldest pose reading we have
  double get_time_min() { return time_min; }

  /// Newest pose reading we have
  double get_time_max() { return time_max; }

private:
  // Our history of POSE messages (time, ori, pos, vel, ang)
  // Note that this is sorted by timestamps so we can binary search through it....
  std::set<POSEDATA, std::less<POSEDATA>> pose_data;

  double time_min = INFINITY;
  double time_max = -INFINITY;
};

#endif /* INTERPOLATOR_H */

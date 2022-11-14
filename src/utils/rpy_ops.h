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
#ifndef RPYOPS_H
#define RPYOPS_H

#include <Eigen/Eigen>

/**
 * @brief Construct rotation matrix from given roll
 * @param t roll angle
 */
static inline Eigen::Matrix3d rot_x(double t) {
  Eigen::Matrix3d r;
  double ct = cos(t);
  double st = sin(t);
  r << 1.0, 0.0, 0.0, 0.0, ct, -st, 0.0, st, ct;
  return r;
}

/**
 * @brief Construct rotation matrix from given pitch
 * @param t pitch angle
 */
static inline Eigen::Matrix3d rot_y(double t) {
  Eigen::Matrix3d r;
  double ct = cos(t);
  double st = sin(t);
  r << ct, 0.0, st, 0.0, 1.0, 0.0, -st, 0.0, ct;
  return r;
}

/**
 * @brief Construct rotation matrix from given yaw
 * @param t yaw angle
 */
static inline Eigen::Matrix3d rot_z(double t) {
  Eigen::Matrix3d r;
  double ct = cos(t);
  double st = sin(t);
  r << ct, -st, 0.0, st, ct, 0.0, 0.0, 0.0, 1.0;
  return r;
}

/**
 * @brief Gets roll, pitch, yaw of argument rotation (in that order).
 * To recover the matrix: R_input = R_z(yaw)*R_y(pitch)*R_x(roll)
 * Rotation Definition: http://planning.cs.uiuc.edu/node102.html
 * Derivation: http://planning.cs.uiuc.edu/node103.html
 * @param rot Rotation matrix
 * @return [roll,pitch,yaw] values
 */
static inline Eigen::Vector3d rot2rpy(const Eigen::Matrix3d &rot) {
  Eigen::Vector3d rpy;
  rpy(2) = atan2(rot(1, 0), rot(0, 0));
  rpy(1) = atan2(-rot(2, 0), sqrt(rot(2, 1) * rot(2, 1) + rot(2, 2) * rot(2, 2)));
  rpy(0) = atan2(rot(2, 1), rot(2, 2));
  return rpy;
}

/**
 * @brief This will ensure that the angle theta is valid and in range [-pi,pi].
 * @param theta Input 1d rotation
 * @return Equivilent rotation within the range [-pi,pi]
 */
static inline double wrap2pi(double theta) {
  while (theta > M_PI)
    theta -= 2 * M_PI;
  while (theta < -M_PI)
    theta += 2 * M_PI;
  return theta;
}

#endif // RPYOPS_H

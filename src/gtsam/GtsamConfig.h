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
#ifndef GTSAMCONFIG_H
#define GTSAMCONFIG_H

#include <Eigen/Eigen>

/**
 * @brief Configuration object that informs the factors if they need to compute Jacobians.
 *
 * For some reason GTSAM doesn't seem to support "fixing" of variables, thus we would need to make a bunch of factors.
 * We instead will always add calibration factors to the graph (which might make it slower), but then just zero the Jacobians.
 * Zero'ing the Jacobians in respect to the calibration will allow them to remain the same (i.e. the factors are not a function of).
 */
struct GtsamConfig {

public:
  /// If we want to estimate the time offset between VICON and IMU
  bool estimate_vicon_imu_toff = true;

  /// If we want to estimate the orientation between VICON and IMU
  bool estimate_vicon_imu_ori = true;

  /// If we want to estimate the position between VICON and IMU
  bool estimate_vicon_imu_pos = true;
};

#endif // GTSAMCONFIG_H
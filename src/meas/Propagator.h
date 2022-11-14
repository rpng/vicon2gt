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
#ifndef PROPAGATOR_H
#define PROPAGATOR_H

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <vector>

#include "cpi/CpiBase.h"
#include "cpi/CpiV1.h"
#include "utils/colors.h"
#include "utils/quat_ops.h"

struct IMUDATA {
  double timestamp;
  Eigen::Vector3d wm;
  Eigen::Vector3d am;
};

class Propagator {

public:
  /// Default constuctor
  Propagator(double sigmaw, double sigmawb, double sigmaa, double sigmaab) {
    this->sigma_w = sigmaw;
    this->sigma_wb = sigmawb;
    this->sigma_a = sigmaa;
    this->sigma_ab = sigmaab;
  }

  /// Our feed function for IMU measurements, will append to our historical vector
  void feed_imu(double timestamp, Eigen::Vector3d wm, Eigen::Vector3d am);

  /// This will propgate the preintegration class between the two requested timesteps
  bool propagate(double time0, double time1, Eigen::Vector3d bg_lin, Eigen::Vector3d ba_lin, CpiV1 &integration);

  /// Checks if we have bounding IMU poses around a given timestamp
  bool has_bounding_imu(double timestamp);

private:
  /**
   * Nice helper function that will linearly interpolate between two imu messages
   * This should be used instead of just "cutting" imu messages that bound the camera times
   * Give better time offset if we use this function....
   */
  IMUDATA interpolate_data(IMUDATA imu_1, IMUDATA imu_2, double timestamp) {
    // time-distance lambda
    double lambda = (timestamp - imu_1.timestamp) / (imu_2.timestamp - imu_1.timestamp);
    // cout << "lambda - " << lambda << endl;
    //  interpolate between the two times
    IMUDATA data;
    data.timestamp = timestamp;
    data.am = (1 - lambda) * imu_1.am + lambda * imu_2.am;
    data.wm = (1 - lambda) * imu_1.wm + lambda * imu_2.wm;
    return data;
  }

  // Our history of IMU messages (time, angular, linear)
  std::vector<IMUDATA> imu_data;

  // Our noises
  double sigma_w;  // gyro white noise
  double sigma_wb; // gyro bias walk
  double sigma_a;  // accel white noise
  double sigma_ab; // accel bias walk
};

#endif /* PROPAGATOR_H */

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
#ifndef SIMULATORPARAMS_H
#define SIMULATORPARAMS_H

#include <Eigen/Eigen>
#include <iostream>
#include <string>
#include <vector>

#include "utils/colors.h"
#include "utils/quat_ops.h"
#include "utils/rpy_ops.h"

using namespace std;

/**
 * @brief Struct which stores all options needed for simulation estimation.
 */
struct SimulatorParams {

  // NOISE / CHI2 ============================

  /// Gyroscope white noise (rad/s/sqrt(hz))
  double sigma_w = 1.6968e-04;

  /// Gyroscope random walk (rad/s^2/sqrt(hz))
  double sigma_wb = 1.9393e-05;

  /// Accelerometer white noise (m/s^2/sqrt(hz))
  double sigma_a = 2.0000e-3;

  /// Accelerometer random walk (m/s^3/sqrt(hz))
  double sigma_ab = 3.0000e-03;

  /// Vicon pose noise for orientation (rad) and position (meters)
  Eigen::Matrix<double, 6, 1> sigma_vicon_pose = (Eigen::Matrix<double, 6, 1>() << 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2).finished();

  /**
   * @brief This function will print out all noise parameters loaded.
   * This allows for visual checking that everything was loaded properly from ROS/CMD parsers.
   */
  void print_noise() {
    printf(CYAN "NOISE IMU:\n");
    printf(CYAN "\t- gyroscope_noise_density: %.6f\n", sigma_w);
    printf(CYAN "\t- accelerometer_noise_density: %.5f\n", sigma_a);
    printf(CYAN "\t- gyroscope_random_walk: %.7f\n", sigma_wb);
    printf(CYAN "\t- accelerometer_random_walk: %.6f\n", sigma_ab);
    printf(CYAN "NOISE VICON:\n");
    printf(CYAN "\t- orientation: %.6f, %.6f, %.6f\n", sigma_vicon_pose(0), sigma_vicon_pose(1), sigma_vicon_pose(2));
    printf(CYAN "\t- position: %.6f, %.6f, %.6f\n", sigma_vicon_pose(3), sigma_vicon_pose(4), sigma_vicon_pose(5));
  }

  // STATE DEFAULTS ==========================

  /// Gravity magnitude in the global frame (i.e. 9.81)
  double gravity_magnitude = 9.81;

  /// Rotation between vicon and gravity aligned frame (roll and pitch)
  Eigen::Matrix3d R_GtoV = Eigen::Matrix3d::Identity();

  /// Time offset between vicon and IMU.
  double viconimu_dt = 0.0;

  /// Rotation between vicon marker body and IMU
  Eigen::Matrix3d R_BtoI = Eigen::Matrix3d::Identity();

  /// Position of vicon marker body in IMU
  Eigen::Vector3d p_BinI = Eigen::Vector3d::Zero();

  /**
   * @brief This function will print out all simulated parameters loaded.
   * This allows for visual checking that everything was loaded properly from ROS/CMD parsers.
   */
  void print_state() {
    printf(CYAN "STATE PARAMETERS:\n");
    printf(CYAN "\t- gravity: %.3f\n", gravity_magnitude);
    Eigen::Vector3d rpy = rot2rpy(R_GtoV);
    printf(CYAN "\t- R_GtoV rpy: %.4f, %.4f, %.4f\n", rpy(0), rpy(1), rpy(2));
    printf(CYAN "\t- viconimu_dt: %.4f\n", viconimu_dt);
    Eigen::Vector4d q_BtoI = rot_2_quat(R_BtoI);
    printf(CYAN "\t- q_BtoI: %.3f, %.3f, %.3f, %.3f\n", q_BtoI(0), q_BtoI(1), q_BtoI(2), q_BtoI(3));
    printf(CYAN "\t- p_BinI: %.3f, %.3f, %.3f\n", p_BinI(0), p_BinI(1), p_BinI(2));
  }

  // SIMULATOR ===============================

  /// Path to the trajectory we will b-spline and simulate on. Should be time(s),pos(xyz),ori(xyzw) format.
  string sim_traj_path = "../ov_data/sim/udel_gore.txt";

  /// Frequency (Hz) that we will simulate our inertial measurement unit
  double sim_freq_imu = 400.0;

  /// Frequency (Hz) that we will simulate our camera sensor
  double sim_freq_cam = 10.0;

  /// Frequency (Hz) that we will simulate our vicon sensor
  double sim_freq_vicon = 100.0;

  /// This should be incremented for each run in the Monte-Carlo simulation to generate the same true measurements, but different noise
  /// values.
  int seed = 0;

  /**
   * @brief This function will print out all simulated parameters loaded.
   * This allows for visual checking that everything was loaded properly from ROS/CMD parsers.
   */
  void print_simulation() {
    printf(CYAN "SIMULATION PARAMETERS:\n");
    printf(BOLDRED "\t- seed: %d \n" RESET, seed);
    printf(CYAN "\t- traj path: %s\n", sim_traj_path.c_str());
    printf(CYAN "\t- imu feq: %.2f\n", sim_freq_imu);
    printf(CYAN "\t- cam feq: %.2f\n", sim_freq_cam);
    printf(CYAN "\t- vicon feq: %.2f\n", sim_freq_vicon);
  }
};

#endif // SIMULATORPARAMS_H
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
#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>

#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include "sim/BsplineSE3.h"
#include "sim/SimulatorParams.h"
#include "utils/colors.h"
#include "utils/quat_ops.h"
#include "utils/rpy_ops.h"

/**
 * @brief Master simulator class that generated vicon-inertial measurements
 *
 * Given a trajectory this will generate a SE(3) @ref ov_core::BsplineSE3 for that trajectory.
 * This allows us to get the inertial measurement information at each timestep during this trajectory.
 * We inject bias and white noises into our inertial readings while adding our white noise to the uv measurements also.
 *
 */
class Simulator {

public:
  /**
   * @brief Default constructor, will load all configuration variables
   * @param params_ SimulationParams parameters. Should have already been loaded from cmd.
   */
  Simulator(const SimulatorParams &params_);

  /**
   * @brief Returns if we are actively simulating
   * @return True if we still have simulation data
   */
  bool ok() { return is_running; }

  /**
   * @brief Returns simulator parameters
   */
  SimulatorParams get_params() { return params; }

  /**
   * @brief Get the simulation state at a specified timestep
   * @param desired_time Timestamp we want to get the state at
   * @param imustate State in the MSCKF ordering: [time(sec),q_VtoI,p_IinV,v_IinV,b_gyro,b_accel]
   * @return True if we have a state
   */
  bool get_state_in_vicon(double desired_time, Eigen::Matrix<double, 17, 1> &imustate);

  /**
   * @brief Gets the next IMU reading if we have one.
   * @param time_imu Time that this measurement occurred at
   * @param wm Angular velocity measurement in the inertial frame
   * @param am Linear velocity in the inertial frame
   * @return True if we have a measurement
   */
  bool get_next_imu(double &time_imu, Eigen::Vector3d &wm, Eigen::Vector3d &am);

  /**
   * @brief Gets the next CAMERA reading if we have one.
   * @param time_vicon Time that this measurement occurred at
   * @return True if we have a measurement
   */
  bool get_next_cam(double &time_camera);

  /**
   * @brief Gets the next VICON reading if we have one.
   * @param time_vicon Time that this measurement occurred at
   * @param q_VtoB Rotation from vicon frame to the vicon marker body frame
   * @param p_BinV Position of vicon marker body frame in vicon frame
   * @return True if we have a measurement
   */
  bool get_next_vicon(double &time_vicon, Eigen::Vector4d &q_VtoB, Eigen::Vector3d &p_BinV);

protected:
  /**
   * @brief This will load the trajectory into memory.
   * @param path_traj Path to the trajectory file that we want to read in.
   */
  void load_data(std::string path_traj);

  //===================================================================
  // Configuration variables
  //===================================================================

  /// Parameters that we use to generate our simulation
  SimulatorParams params;

  //===================================================================
  // State related variables
  //===================================================================

  /// Our loaded trajectory data (timestamp(s), q_GtoI, p_IinG)
  std::vector<Eigen::VectorXd> traj_data;

  /// Our b-spline trajectory
  BsplineSE3 spline;

  /// Mersenne twister PRNG for measurements (IMU)
  std::mt19937 gen_meas_imu;

  /// Mersenne twister PRNG for measurements (VICON)
  std::mt19937 gen_meas_vicon;

  /// Mersenne twister PRNG for state perturbations
  std::mt19937 gen_state_perturb;

  /// If our simulation is running
  bool is_running;

  //===================================================================
  // Simulation specific variables
  //===================================================================

  /// Current timestamp of the system
  double timestamp;

  /// Last time we had an IMU reading
  double timestamp_last_imu;

  /// Last time we had an CAMERA reading
  double timestamp_last_cam;

  /// Last time we had an VICON reading
  double timestamp_last_vicon;

  /// Our running acceleration bias
  Eigen::Vector3d true_bias_accel = Eigen::Vector3d::Zero();

  /// Our running gyroscope bias
  Eigen::Vector3d true_bias_gyro = Eigen::Vector3d::Zero();

  // Our history of true biases
  std::vector<double> hist_true_bias_time;
  std::vector<Eigen::Vector3d> hist_true_bias_accel;
  std::vector<Eigen::Vector3d> hist_true_bias_gyro;
};

#endif // SIMULATOR_H

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
#include "Simulator.h"

Simulator::Simulator(const SimulatorParams &params_) {

  //===============================================================
  //===============================================================

  // Generate a random extrinsic and time offset
  // These will be our groundtruth parameters!
  this->params = params_;
  std::normal_distribution<double> w(0, 1);
  std::mt19937 r(params.seed);
  Eigen::Vector3d w_vec;
  w_vec << 0.1 * w(r), 0.1 * w(r), 0.1 * w(r);
  params.R_BtoI = exp_so3(w_vec);
  params.p_BinI << 0.2 * w(r), 0.2 * w(r), 0.2 * w(r);
  params.viconimu_dt = 0.08 * w(r);

  // Random gravity and vicon frame alignment
  params.R_GtoV = rot_y(0.1 * M_PI * w(r)) * rot_x(0.1 * M_PI * w(r));

  //===============================================================
  //===============================================================

  // Nice startup message
  printf(CYAN "=======================================\n");
  printf(CYAN "VICON-INERTIAL SIMULATOR STARTING\n");
  printf(CYAN "=======================================\n");
  params.print_noise();
  params.print_state();
  params.print_simulation();

  // Load the groundtruth trajectory and its spline
  load_data(params.sim_traj_path);
  spline.feed_trajectory(traj_data);

  // Set all our timestamps as starting from the minimum spline time
  timestamp = spline.get_start_time();
  timestamp_last_imu = spline.get_start_time();
  timestamp_last_cam = spline.get_start_time();
  timestamp_last_vicon = spline.get_start_time();

  // Get the pose at the current timestep
  Eigen::Matrix3d R_GtoI_init;
  Eigen::Vector3d p_IinG_init;
  bool success_pose_init = spline.get_pose(timestamp, R_GtoI_init, p_IinG_init);
  if (!success_pose_init) {
    printf(RED "[SIM]: unable to find the first pose in the spline...\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // Append the current true bias to our history
  hist_true_bias_time.push_back(timestamp_last_imu - 1.0 / params.sim_freq_imu);
  hist_true_bias_accel.push_back(true_bias_accel);
  hist_true_bias_gyro.push_back(true_bias_gyro);
  hist_true_bias_time.push_back(timestamp_last_imu);
  hist_true_bias_accel.push_back(true_bias_accel);
  hist_true_bias_gyro.push_back(true_bias_gyro);

  // Our simulation is running
  is_running = true;

  //===============================================================
  //===============================================================

  // Load the seeds for the random number generators
  gen_meas_imu = std::mt19937(params.seed);
  gen_meas_imu.seed(params.seed);
  gen_meas_vicon = std::mt19937(params.seed);
  gen_meas_vicon.seed(params.seed);
}

bool Simulator::get_state_in_vicon(double desired_time, Eigen::Matrix<double, 17, 1> &imustate) {

  // Set to default state
  imustate.setZero();
  imustate(4) = 1;

  // Current state values
  Eigen::Matrix3d R_GtoI;
  Eigen::Vector3d p_IinG, w_IinI, v_IinG;

  // Get the pose, velocity, and acceleration
  bool success_vel = spline.get_velocity(desired_time, R_GtoI, p_IinG, w_IinI, v_IinG);

  // Find the bounding bias values
  bool success_bias = false;
  size_t id_loc = 0;
  for (size_t i = 0; i < hist_true_bias_time.size() - 1; i++) {
    if (hist_true_bias_time.at(i) < desired_time && hist_true_bias_time.at(i + 1) >= desired_time) {
      id_loc = i;
      success_bias = true;
      break;
    }
  }

  // If failed, then that means we don't have any more spline or bias
  if (!success_vel || !success_bias) {
    return false;
  }

  // Interpolate our biases (they will be at every IMU timestep)
  double lambda = (desired_time - hist_true_bias_time.at(id_loc)) / (hist_true_bias_time.at(id_loc + 1) - hist_true_bias_time.at(id_loc));
  Eigen::Vector3d true_bg_interp = (1 - lambda) * hist_true_bias_gyro.at(id_loc) + lambda * hist_true_bias_gyro.at(id_loc + 1);
  Eigen::Vector3d true_ba_interp = (1 - lambda) * hist_true_bias_accel.at(id_loc) + lambda * hist_true_bias_accel.at(id_loc + 1);

  // Finally lets create the current state
  imustate(0, 0) = desired_time;
  imustate.block(1, 0, 4, 1) = rot_2_quat(R_GtoI * params.R_GtoV.transpose());
  imustate.block(5, 0, 3, 1) = params.R_GtoV * p_IinG;
  imustate.block(8, 0, 3, 1) = params.R_GtoV * v_IinG;
  imustate.block(11, 0, 3, 1) = true_bg_interp;
  imustate.block(14, 0, 3, 1) = true_ba_interp;
  return true;
}

bool Simulator::get_next_imu(double &time_imu, Eigen::Vector3d &wm, Eigen::Vector3d &am) {

  // Return if the camera measurement should go before us
  if (timestamp_last_cam + 1.0 / params.sim_freq_cam < timestamp_last_imu + 1.0 / params.sim_freq_imu ||
      timestamp_last_vicon + 1.0 / params.sim_freq_vicon < timestamp_last_imu + 1.0 / params.sim_freq_imu)
    return false;

  // Else lets do a new measurement!!!
  timestamp_last_imu += 1.0 / params.sim_freq_imu;
  timestamp = timestamp_last_imu;
  time_imu = timestamp_last_imu;

  // Current state values
  Eigen::Matrix3d R_GtoI;
  Eigen::Vector3d p_IinG, w_IinI, v_IinG, alpha_IinI, a_IinG;

  // Get the pose, velocity, and acceleration
  // NOTE: we get the acceleration between our two IMU
  // NOTE: this is because we are using a constant measurement model for integration
  // bool success_accel = spline.get_acceleration(timestamp+0.5/freq_imu, R_GtoI, p_IinG, w_IinI, v_IinG, alpha_IinI, a_IinG);
  bool success_accel = spline.get_acceleration(timestamp, R_GtoI, p_IinG, w_IinI, v_IinG, alpha_IinI, a_IinG);

  // If failed, then that means we don't have any more spline
  // Thus we should stop the simulation
  if (!success_accel) {
    is_running = false;
    return false;
  }

  // Transform omega and linear acceleration into imu frame
  Eigen::Vector3d omega_inI = w_IinI;
  Eigen::Vector3d ez = {0, 0, 1};
  Eigen::Vector3d accel_inI = R_GtoI * (a_IinG + params.gravity_magnitude * ez);

  // Now add noise to these measurements
  double dt = 1.0 / params.sim_freq_imu;
  std::normal_distribution<double> w(0, 1);
  wm(0) = omega_inI(0) + true_bias_gyro(0) + params.sigma_w / std::sqrt(dt) * w(gen_meas_imu);
  wm(1) = omega_inI(1) + true_bias_gyro(1) + params.sigma_w / std::sqrt(dt) * w(gen_meas_imu);
  wm(2) = omega_inI(2) + true_bias_gyro(2) + params.sigma_w / std::sqrt(dt) * w(gen_meas_imu);
  am(0) = accel_inI(0) + true_bias_accel(0) + params.sigma_a / std::sqrt(dt) * w(gen_meas_imu);
  am(1) = accel_inI(1) + true_bias_accel(1) + params.sigma_a / std::sqrt(dt) * w(gen_meas_imu);
  am(2) = accel_inI(2) + true_bias_accel(2) + params.sigma_a / std::sqrt(dt) * w(gen_meas_imu);

  // Move the biases forward in time
  true_bias_gyro(0) += params.sigma_wb * std::sqrt(dt) * w(gen_meas_imu);
  true_bias_gyro(1) += params.sigma_wb * std::sqrt(dt) * w(gen_meas_imu);
  true_bias_gyro(2) += params.sigma_wb * std::sqrt(dt) * w(gen_meas_imu);
  true_bias_accel(0) += params.sigma_ab * std::sqrt(dt) * w(gen_meas_imu);
  true_bias_accel(1) += params.sigma_ab * std::sqrt(dt) * w(gen_meas_imu);
  true_bias_accel(2) += params.sigma_ab * std::sqrt(dt) * w(gen_meas_imu);

  // Append the current true bias to our history
  hist_true_bias_time.push_back(timestamp_last_imu);
  hist_true_bias_gyro.push_back(true_bias_gyro);
  hist_true_bias_accel.push_back(true_bias_accel);

  // Return success
  return true;
}

bool Simulator::get_next_cam(double &time_camera) {

  // Return if the camera measurement should go before us
  if (timestamp_last_imu + 1.0 / params.sim_freq_imu < timestamp_last_cam + 1.0 / params.sim_freq_cam ||
      timestamp_last_vicon + 1.0 / params.sim_freq_vicon < timestamp_last_cam + 1.0 / params.sim_freq_cam)
    return false;

  // Else lets do a new measurement!!!
  timestamp_last_cam += 1.0 / params.sim_freq_cam;
  timestamp = timestamp_last_cam;
  time_camera = timestamp_last_cam;

  // Return success
  return true;
}

bool Simulator::get_next_vicon(double &time_vicon, Eigen::Vector4d &q_VtoB, Eigen::Vector3d &p_BinV) {

  // Return if the camera measurement should go before us
  if (timestamp_last_imu + 1.0 / params.sim_freq_imu < timestamp_last_vicon + 1.0 / params.sim_freq_vicon ||
      timestamp_last_cam + 1.0 / params.sim_freq_cam < timestamp_last_vicon + 1.0 / params.sim_freq_vicon)
    return false;

  // Else lets do a new measurement!!!
  timestamp_last_vicon += 1.0 / params.sim_freq_vicon;
  timestamp = timestamp_last_vicon;
  time_vicon = timestamp_last_vicon - params.viconimu_dt;

  // Get the pose at the current timestep
  Eigen::Matrix3d R_GtoI;
  Eigen::Vector3d p_IinG;
  bool success_pose = spline.get_pose(timestamp, R_GtoI, p_IinG);

  // We have finished generating measurements
  if (!success_pose) {
    is_running = false;
    return false;
  }

  // Transform into the vicon marker frame
  Eigen::Matrix3d R_GtoB = params.R_BtoI.transpose() * R_GtoI;
  Eigen::Vector3d p_BinG = p_IinG + R_GtoI.transpose() * params.p_BinI;

  // Rotate into the vicon frame from the inertial frame
  // NOTE: for simulation we keep the gravity aligned frame fixed
  // NOTE: while the vicon frame is changed, this allows for evaulation
  // NOTE: of many runs with different vicon frames to be compared in the global gravity frame
  // NOTE: ie. the groundtruth bspline trajectory is in the global inertial frame (not the vicon!!)
  Eigen::Matrix3d tmp_R_VtoB = R_GtoB * params.R_GtoV.transpose();
  Eigen::Vector3d tmp_p_BinV = params.R_GtoV * p_BinG;

  // Inject noise also flip the position direction
  std::normal_distribution<double> w(0, 1);
  Eigen::Vector3d w_vec = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_vec = Eigen::Vector3d::Zero();
  w_vec << params.sigma_vicon_pose(0) * w(gen_meas_vicon), params.sigma_vicon_pose(1) * w(gen_meas_vicon),
      params.sigma_vicon_pose(2) * w(gen_meas_vicon);
  p_vec << params.sigma_vicon_pose(3) * w(gen_meas_vicon), params.sigma_vicon_pose(4) * w(gen_meas_vicon),
      params.sigma_vicon_pose(5) * w(gen_meas_vicon);

  // Set our measurement
  q_VtoB = rot_2_quat(exp_so3(w_vec) * tmp_R_VtoB);
  p_BinV = tmp_p_BinV + p_vec;

  // Return success
  return true;
}

void Simulator::load_data(std::string path_traj) {

  // Try to open our groundtruth file
  std::ifstream file;
  file.open(path_traj);
  if (!file) {
    printf(RED "ERROR: Unable to open simulation trajectory file...\n" RESET);
    printf(RED "ERROR: %s\n" RESET, path_traj.c_str());
    std::exit(EXIT_FAILURE);
  }

  // Debug print
  std::string base_filename = path_traj.substr(path_traj.find_last_of("/\\") + 1);
  printf("[SIM]: loaded trajectory %s\n", base_filename.c_str());

  // Loop through each line of this file
  std::string current_line;
  while (std::getline(file, current_line)) {

    // Skip if we start with a comment
    if (!current_line.find("#"))
      continue;

    // Loop variables
    int i = 0;
    std::istringstream s(current_line);
    std::string field;
    Eigen::Matrix<double, 8, 1> data;

    // Loop through this line (timestamp(s) tx ty tz qx qy qz qw)
    while (std::getline(s, field, ' ')) {
      // Skip if empty
      if (field.empty() || i >= data.rows())
        continue;
      // save the data to our vector
      data(i) = std::atof(field.c_str());
      i++;
    }

    // Only a valid line if we have all the parameters
    if (i > 7) {
      traj_data.push_back(data);
      // std::cout << std::setprecision(15) << data.transpose() << std::endl;
    }
  }

  // Finally close the file
  file.close();

  // Error if we don't have any data
  if (traj_data.empty()) {
    printf(RED "ERROR: Could not parse any data from the file!!\n" RESET);
    printf(RED "ERROR: %s\n" RESET, path_traj.c_str());
    std::exit(EXIT_FAILURE);
  }
}

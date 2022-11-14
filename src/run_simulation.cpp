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

#include <Eigen/Eigen>
#include <cmath>
#include <memory>
#include <unistd.h>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>

#include "meas/Interpolator.h"
#include "meas/Propagator.h"
#include "sim/Simulator.h"
#include "solver/ViconGraphSolver.h"
#include "utils/stats.h"

int main(int argc, char **argv) {

  // Start up
  ros::init(argc, argv, "run_simulation");
  ros::NodeHandle nh("~");

  // Setup publisher (needs to be at top so ROS registers it)
  ros::Publisher pub_pathgt = nh.advertise<nav_msgs::Path>("/vicon2gt/groundtruth", 2);

  // Load the export path
  bool save2file;
  std::string path_states, path_states_gt, path_info;
  int state_freq;
  nh.param<std::string>("stats_path_states", path_states, "states.csv");
  nh.param<std::string>("stats_path_states_gt", path_states_gt, "gt.csv");
  nh.param<std::string>("stats_path_info", path_info, "vicon2gt_info.txt");
  nh.param<bool>("save2file", save2file, false);
  nh.param<int>("state_freq", state_freq, 100);
  ROS_INFO("save path information...");
  ROS_INFO("    - state path: %s", path_states.c_str());
  ROS_INFO("    - info path: %s", path_info.c_str());
  ROS_INFO("    - save to file: %d", (int)save2file);
  ROS_INFO("    - state_freq: %d", state_freq);

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Our simulator params
  SimulatorParams params;

  // Simulation params
  nh.param<std::string>("sim_traj_path", params.sim_traj_path, params.sim_traj_path);
  nh.param<double>("sim_freq_imu", params.sim_freq_imu, params.sim_freq_imu);
  nh.param<double>("sim_freq_cam", params.sim_freq_cam, params.sim_freq_cam);
  nh.param<double>("sim_freq_vicon", params.sim_freq_vicon, params.sim_freq_vicon);
  nh.param<int>("sim_seed", params.seed, params.seed);

  // Our IMU noise values
  double sigma_w, sigma_wb, sigma_a, sigma_ab;
  nh.param<double>("gyroscope_noise_density", sigma_w, 1.6968e-04);
  nh.param<double>("accelerometer_noise_density", sigma_a, 2.0000e-3);
  nh.param<double>("gyroscope_random_walk", sigma_wb, 1.9393e-05);
  nh.param<double>("accelerometer_random_walk", sigma_ab, 3.0000e-03);

  // Vicon sigmas (used if we don't have odometry messages)
  Eigen::Matrix3d R_q = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d R_p = Eigen::Matrix3d::Zero();
  std::vector<double> viconsigmas;
  std::vector<double> viconsigmas_default = {1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2};
  nh.param<std::vector<double>>("vicon_sigmas", viconsigmas, viconsigmas_default);
  R_q(0, 0) = std::pow(viconsigmas.at(0), 2);
  R_q(1, 1) = std::pow(viconsigmas.at(1), 2);
  R_q(2, 2) = std::pow(viconsigmas.at(2), 2);
  R_p(0, 0) = std::pow(viconsigmas.at(3), 2);
  R_p(1, 1) = std::pow(viconsigmas.at(4), 2);
  R_p(2, 2) = std::pow(viconsigmas.at(5), 2);
  params.sigma_vicon_pose << viconsigmas.at(0), viconsigmas.at(1), viconsigmas.at(2), viconsigmas.at(3), viconsigmas.at(4),
      viconsigmas.at(5);

  // Load gravity magnitude
  nh.param<double>("gravity_magnitude", params.gravity_magnitude, 9.81);

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Our simulator
  std::shared_ptr<Simulator> sim = std::make_shared<Simulator>(params);

  // Our data storage objects
  std::shared_ptr<Propagator> propagator = std::make_shared<Propagator>(sigma_w, sigma_wb, sigma_a, sigma_ab);
  std::shared_ptr<Interpolator> interpolator = std::make_shared<Interpolator>();

  // Counts on how many measurements we have
  int ct_imu = 0;
  int ct_vic = 0;
  double start_time = -1;
  double end_time = -1;

  // Step through the rosbag
  while (sim->ok() && ros::ok()) {

    // IMU: get the next simulated IMU measurement if we have it
    double time_imu;
    Eigen::Vector3d wm, am;
    if (sim->get_next_imu(time_imu, wm, am)) {
      propagator->feed_imu(time_imu, wm, am);
      ct_imu++;
    }

    // CAM: get the next simulated camera uv measurements if we have them
    double time_cam;
    sim->get_next_cam(time_cam);

    // VICON: get the next simulated camera uv measurements if we have them
    double time_vicon;
    Eigen::Vector4d q_VtoB;
    Eigen::Vector3d p_BinV;
    if (sim->get_next_vicon(time_vicon, q_VtoB, p_BinV)) {
      interpolator->feed_pose(time_vicon, q_VtoB, p_BinV, R_q, R_p);
      ct_vic++;
      if (start_time == -1) {
        start_time = time_vicon;
      }
      if (start_time != -1) {
        end_time = time_vicon;
      }
    }
  }

  // Create our camera timestamps at the requested fix frequency
  int ct_cam = 0;
  std::vector<double> timestamp_cameras;
  if (start_time != -1 && end_time != -1 && start_time < end_time) {
    double temp_time = start_time;
    while (temp_time < end_time) {
      timestamp_cameras.push_back(temp_time);
      temp_time += 1.0 / (double)state_freq;
      ct_cam++;
    }
  }

  // Print out how many we have loaded
  ROS_INFO("done loading the rosbag...");
  ROS_INFO("    - number imu   = %d", ct_imu);
  ROS_INFO("    - number cam   = %d", ct_cam);
  ROS_INFO("    - number vicon = %d", ct_vic);

  // Create the graph problem, and solve it
  ViconGraphSolver solver(nh, propagator, interpolator, timestamp_cameras);
  solver.build_and_solve();

  // Visualize onto ROS
  solver.visualize();

  // Finally, save to file all the information
  std::ofstream of_state;
  if (save2file) {

    // save generated trajectory
    solver.write_to_file(path_states, path_info);

    // Open the groundtruth trajectory file
    ROS_INFO("saving *groundtruth* states to file");
    if (boost::filesystem::exists(path_states_gt)) {
      boost::filesystem::remove(path_states_gt);
      ROS_INFO("    - old state file found, deleted...");
    }
    boost::filesystem::path p1(path_states_gt);
    boost::filesystem::create_directories(p1.parent_path());

    // Open our state file!
    of_state.open(path_states_gt, std::ofstream::out | std::ofstream::app);
    of_state << "#time(ns),px,py,pz,qw,qx,qy,qz,vx,vy,vz,bwx,bwy,bwz,bax,bay,baz" << std::endl;
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Get the final optimized poses
  std::vector<double> times;
  std::vector<Eigen::Matrix<double, 10, 1>> poses;
  solver.get_imu_poses(times, poses);

  // Now compute the error compared to our true states
  std::vector<geometry_msgs::PoseStamped> poses_gtimu;
  Stats err_ori, err_pos, err_vel;
  for (size_t i = 0; i < times.size(); i++) {

    // get the states
    // GT: [time(sec),q_VtoI,p_IinV,v_IinV,b_gyro,b_accel]
    // EST: [q_VtoI,p_IinV]
    Eigen::Matrix<double, 17, 1> gt_state;
    sim->get_state_in_vicon(times.at(i), gt_state);
    Eigen::Matrix<double, 10, 1> est_state = poses.at(i);

    // compute error
    double ori = 2.0 * (quat_multiply(gt_state.block(1, 0, 4, 1), Inv(est_state.block(0, 0, 4, 1)))).block(0, 0, 3, 1).norm();
    double pose = (est_state.block(4, 0, 3, 1) - gt_state.block(5, 0, 3, 1)).norm();
    double vel = (est_state.block(7, 0, 3, 1) - gt_state.block(8, 0, 3, 1)).norm();

    // Append to the history
    // cout << i << " " << 180.0/M_PI*ori << " " << pose << endl;
    // cout << gt_state.block(1,0,7,1).transpose() << endl;
    // cout << est_state.transpose() << endl;
    err_ori.timestamps.push_back(times.at(i));
    err_ori.values.push_back(180.0 / M_PI * ori);
    err_pos.timestamps.push_back(times.at(i));
    err_pos.values.push_back(pose);
    err_vel.timestamps.push_back(times.at(i));
    err_vel.values.push_back(vel);

    // Create the ROS pose for visualization
    geometry_msgs::PoseStamped posetemp;
    posetemp.header.stamp = ros::Time(times.at(i));
    posetemp.header.frame_id = "vicon";
    posetemp.pose.orientation.x = gt_state(1);
    posetemp.pose.orientation.y = gt_state(2);
    posetemp.pose.orientation.z = gt_state(3);
    posetemp.pose.orientation.w = gt_state(4);
    posetemp.pose.position.x = gt_state(5);
    posetemp.pose.position.y = gt_state(6);
    posetemp.pose.position.z = gt_state(7);
    poses_gtimu.push_back(posetemp);

    // Write this pose to the gt trajectory file if open
    // GT: [time(sec),q_GtoI,p_IinG,v_IinG,b_gyro,b_accel]
    // CSV: (time(ns),px,py,pz,qw,qx,qy,qz,vx,vy,vz,bwx,bwy,bwz,bax,bay,baz)
    if (of_state.is_open()) {
      Eigen::Vector4d q_GtoV = rot_2_quat(sim->get_params().R_GtoV);
      Eigen::Vector4d q_GtoIi = quat_multiply(gt_state.block(1, 0, 4, 1), q_GtoV);
      Eigen::Vector3d p_IiinG = sim->get_params().R_GtoV.transpose() * gt_state.block(5, 0, 3, 1);
      Eigen::Vector3d v_IiinG = sim->get_params().R_GtoV.transpose() * gt_state.block(8, 0, 3, 1);
      of_state << std::setprecision(20) << std::floor(1e9 * times.at(i)) << "," << std::setprecision(6) << p_IiinG(0) << "," << p_IiinG(1)
               << "," << p_IiinG(2) << "," << q_GtoIi(3) << "," << q_GtoIi(0) << "," << q_GtoIi(1) << "," << q_GtoIi(2) << "," << v_IiinG(0)
               << "," << v_IiinG(1) << "," << v_IiinG(2) << "," << gt_state(11) << "," << gt_state(12) << "," << gt_state(13) << ","
               << gt_state(14) << "," << gt_state(15) << "," << gt_state(16) << std::endl;
    }
  }

  // Close the groundtruth trajectory file if open
  if (of_state.is_open()) {
    of_state.close();
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Tell the user we are publishing
  ROS_INFO("Publishing: %s", pub_pathgt.getTopic().c_str());

  // Create our path (vicon)
  // NOTE: We downsample the number of poses as needed to prevent rviz crashes
  // NOTE: https://github.com/ros-visualization/rviz/issues/1107
  nav_msgs::Path arrVICON;
  arrVICON.header.stamp = ros::Time::now();
  arrVICON.header.frame_id = "vicon";
  for (size_t i = 0; i < poses_gtimu.size(); i += std::floor(poses_gtimu.size() / 16384.0) + 1) {
    arrVICON.poses.push_back(poses_gtimu.at(i));
  }
  pub_pathgt.publish(arrVICON);

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Calculate and print trajectory error
  err_ori.calculate();
  err_pos.calculate();
  err_vel.calculate();
  printf(REDPURPLE "======================================\n");
  printf(REDPURPLE "Trajectory Errors (deg,m,m/s)\n");
  printf(REDPURPLE "======================================\n");
  printf(REDPURPLE "rmse_ori = %.5f | rmse_pos = %.5f | rmse_vel = %.5f\n", err_ori.rmse, err_pos.rmse, err_vel.rmse);
  printf(REDPURPLE "mean_ori = %.5f | mean_pos = %.5f | mean_vel = %.5f\n", err_ori.mean, err_pos.mean, err_vel.mean);
  printf(REDPURPLE "min_ori  = %.5f | min_pos  = %.5f | min_vel  = %.5f\n", err_ori.min, err_pos.min, err_vel.min);
  printf(REDPURPLE "max_ori  = %.5f | max_pos  = %.5f | max_vel  = %.5f\n", err_ori.max, err_pos.max, err_vel.max);
  printf(REDPURPLE "std_ori  = %.5f | std_pos  = %.5f | std_vel  = %.5f\n\n", err_ori.std, err_pos.std, err_vel.std);

  // Get converged calibration
  double toff;
  Eigen::Matrix3d R_BtoI, R_GtoV;
  Eigen::Vector3d p_BinI;
  solver.get_calibration(toff, R_BtoI, p_BinI, R_GtoV);
  Eigen::Vector4d q_BtoI = rot_2_quat(R_BtoI);
  Eigen::Vector4d gt_q_BtoI = rot_2_quat(sim->get_params().R_BtoI);

  printf(REDPURPLE "======================================\n");
  printf(REDPURPLE "Converged Calib vs Groundtruth\n");
  printf(REDPURPLE "======================================\n");
  printf(REDPURPLE "GT  toff: %.5f\n", sim->get_params().viconimu_dt);
  printf(REDPURPLE "EST toff: %.5f\n\n", toff);
  printf(REDPURPLE "GT  rpy R_GtoV: %.3f, %.3f, %.3f\n", rot2rpy(sim->get_params().R_GtoV)(0), rot2rpy(sim->get_params().R_GtoV)(1),
         rot2rpy(sim->get_params().R_GtoV)(2));
  printf(REDPURPLE "EST rpy R_GtoV: %.3f, %.3f, %.3f\n\n", rot2rpy(R_GtoV)(0), rot2rpy(R_GtoV)(1), rot2rpy(R_GtoV)(2));
  printf(REDPURPLE "GT  q_BtoI: %.3f, %.3f, %.3f, %.3f\n", gt_q_BtoI(0), gt_q_BtoI(1), gt_q_BtoI(2), gt_q_BtoI(3));
  printf(REDPURPLE "EST q_BtoI: %.3f, %.3f, %.3f, %.3f\n\n", q_BtoI(0), q_BtoI(1), q_BtoI(2), q_BtoI(3));
  printf(REDPURPLE "GT  p_BinI: %.3f, %.3f, %.3f\n", sim->get_params().p_BinI(0), sim->get_params().p_BinI(1), sim->get_params().p_BinI(2));
  printf(REDPURPLE "EST p_BinI: %.3f, %.3f, %.3f\n\n", p_BinI(0), p_BinI(1), p_BinI(2));

  // Done!
  return EXIT_SUCCESS;
}

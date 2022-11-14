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
#ifndef VICONGRAPHSOLVER_H
#define VICONGRAPHSOLVER_H

#include <Eigen/Eigen>
#include <fstream>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <vector>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include "cpi/CpiV1.h"
#include "gtsam/GtsamConfig.h"
#include "gtsam/ImuFactorCPIv1.h"
#include "gtsam/JPLNavState.h"
#include "gtsam/JPLQuaternion.h"
#include "gtsam/MeasBased_ViconPoseTimeoffsetFactor.h"
#include "gtsam/RotationXY.h"
#include "meas/Interpolator.h"
#include "meas/Propagator.h"
#include "utils/quat_ops.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>

using namespace std;
using namespace gtsam;

using gtsam::symbol_shorthand::C; // C: calibration (c(0)=rot, c(1)=pos)
using gtsam::symbol_shorthand::G; // G: global gravity rotation into the vicon frame
using gtsam::symbol_shorthand::T; // T: time offset between vicon and imu sensors
using gtsam::symbol_shorthand::X; // X: our JPL states

class ViconGraphSolver {

public:
  /**
   * @brief Default constructor for the solver
   * @param nh ROS node handler we will load parameters from
   * @param propagator Propagator with all IMU measurements inside
   * @param interpolator Interpolator with all vicon poses inside
   * @param timestamp_cameras Timestamps we are interested in estimating
   */
  ViconGraphSolver(ros::NodeHandle &nh, std::shared_ptr<Propagator> propagator, std::shared_ptr<Interpolator> interpolator,
                   std::vector<double> timestamp_cameras);

  /**
   * @brief This will build the graph and solve it.
   * This function will take a while, but handles the GTSAM optimization.
   */
  void build_and_solve();

  /**
   * @brief Will export the graph to csv file (will be in eth format).
   *
   * The CSV file will be in the eth format:
   * `(time(ns),px,py,pz,qw,qx,qy,qz,vx,vy,vz,bwx,bwy,bwz,bax,bay,baz)`
   *
   * @param csvfilepath CSV export file we want to save
   * @param infofilepath Txt file we will save the found calibration parameters
   */
  void write_to_file(std::string csvfilepath, std::string infofilepath);

  /**
   * @brief Will publish the trajectories onto ROS for visualization in RVIZ
   */
  void visualize();

  /**
   * @brief Returns the current optimized poses which we estimated
   * @param times Timestamps in seconds each pose will occur at
   * @param poses Poses in quaternion position velocity ordering
   */
  void get_imu_poses(std::vector<double> &times, std::vector<Eigen::Matrix<double, 10, 1>> &poses);

  /**
   * @brief Gets other calibration parameters we estimate online
   * @param toff Time offset between vicon and IMU
   * @param R_BtoI Rotation between vicon and IMU
   * @param p_BinI Position between vicon and IMU
   * @param R_GtoV Rotation from gravity aligned to vicon frame
   */
  void get_calibration(double &toff, Eigen::Matrix3d &R_BtoI, Eigen::Vector3d &p_BinI, Eigen::Matrix3d &R_GtoV);

protected:
  /**
   * @brief This will build the graph problem and add all measurements and nodes to it
   * Given the first time, we init the states using the VICON, but in the future we keep them
   * And only re-linearize measurements (i.e. for the preintegration biases)
   * @param init_states If true we will append the nodes to the gtsam problem.
   */
  void build_problem(bool init_states);

  /**
   * @brief This will optimize the graph.
   * Uses Levenberg-Marquardt for the optimization.
   */
  void optimize_problem();

  // Timing variables
  boost::posix_time::ptime rT1, rT2, rT3, rT4, rT5, rT6, rT7;

  // ROS node handler
  ros::NodeHandle nh;
  ros::Publisher pub_pathimu, pub_pathvicon, pub_vicon_raw;
  double vicon_raw_pub_freq;

  // Measurement data from the rosbag
  std::shared_ptr<Propagator> propagator;
  std::shared_ptr<Interpolator> interpolator;
  std::vector<double> timestamp_cameras;

  // Initial estimates of our variables
  Eigen::Matrix3d init_R_GtoV;
  Eigen::Matrix3d init_R_BtoI;
  Eigen::Vector3d init_p_BinI;
  double init_toff_imu_to_vicon;

  // We do not optimize the gravity magnitude
  double gravity_magnitude;

  // Master non-linear GTSAM graph, all created factors
  // Also have all nodes in the graph
  gtsam::NonlinearFactorGraph *graph;
  gtsam::Values values;

  // Optimized values
  gtsam::Values values_result;

  // Config for what we are optimizing
  std::shared_ptr<GtsamConfig> config;

  // Map between state timestamp and their IDs
  std::map<double, size_t> map_states;

  // Number of times we will loop and relinearize the measurements
  int num_loop_relin;

  // Small dt we will perturb to do our time derivative of
  double TIME_OFFSET = 0.25;
};

#endif /* VICONGRAPHSOLVER_H */

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
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include "meas/Interpolator.h"
#include "meas/Propagator.h"
#include "solver/ViconGraphSolver.h"

int main(int argc, char **argv) {

  // Start up
  ros::init(argc, argv, "estimate_vicon2gt");
  ros::NodeHandle nh("~");
  auto rT1 = boost::posix_time::microsec_clock::local_time();

  // Load the imu, camera, and vicon topics
  std::string topic_imu, topic_vicon;
  nh.param<std::string>("topic_imu", topic_imu, "/imu0");
  nh.param<std::string>("topic_vicon", topic_vicon, "/vicon/ironsides/odom");

  // Load the bag path
  bool save_to_file, use_manual_sigmas;
  std::string path_to_bag, path_states, path_info;
  int state_freq;
  nh.param<std::string>("path_bag", path_to_bag, "bagfile.bag");
  nh.param<std::string>("stats_path_states", path_states, "gt_states.csv");
  nh.param<std::string>("stats_path_info", path_info, "vicon2gt_info.txt");
  nh.param<bool>("save_to_file", save_to_file, save_to_file);
  nh.param<bool>("use_manual_sigmas", use_manual_sigmas, false);
  nh.param<int>("state_freq", state_freq, 100);
  ROS_INFO("rosbag information...");
  ROS_INFO("    - bag path: %s", path_to_bag.c_str());
  ROS_INFO("    - state path: %s", path_states.c_str());
  ROS_INFO("    - info path: %s", path_info.c_str());
  ROS_INFO("    - save to file: %d", (int)save_to_file);
  ROS_INFO("    - use manual sigmas: %d", (int)use_manual_sigmas);
  ROS_INFO("    - state_freq: %d", state_freq);

  // Get our start location and how much of the bag we want to play
  // Make the bag duration < 0 to just process to the end of the bag
  double bag_start, bag_durr;
  nh.param<double>("bag_start", bag_start, 0);
  nh.param<double>("bag_durr", bag_durr, -1);

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Load rosbag here, and find messages we can play
  rosbag::Bag bag;
  bag.open(path_to_bag, rosbag::bagmode::Read);

  // We should load the bag as a view
  // Here we go from beginning of the bag to the end of the bag
  rosbag::View view_full;
  rosbag::View view;

  // Start a few seconds in from the full view time
  // If we have a negative duration then use the full bag length
  view_full.addQuery(bag, rosbag::TopicQuery({topic_imu, topic_vicon}));
  ros::Time time_init = view_full.getBeginTime();
  time_init += ros::Duration(bag_start);
  ros::Time time_finish = (bag_durr < 0) ? view_full.getEndTime() : time_init + ros::Duration(bag_durr);
  ROS_INFO("loading rosbag into memory...");
  ROS_INFO("    - time start = %.6f", time_init.toSec());
  ROS_INFO("    - time end   = %.6f", time_finish.toSec());
  ROS_INFO("    - duration   = %.2f (secs)", time_finish.toSec() - time_init.toSec());
  view.addQuery(bag, rosbag::TopicQuery({topic_imu, topic_vicon}), time_init, time_finish);

  // Check to make sure we have data to play
  if (view.size() == 0) {
    ROS_ERROR("No messages to play on specified topics.  Exiting.");
    ROS_ERROR("IMU TOPIC: %s", topic_imu.c_str());
    ROS_ERROR("VIC TOPIC: %s", topic_vicon.c_str());
    ros::shutdown();
    return EXIT_FAILURE;
  }
  auto rT2 = boost::posix_time::microsec_clock::local_time();
  ROS_INFO("\u001b[34m[TIME]: %.4f to load bag\u001b[0m", (rT2 - rT1).total_microseconds() * 1e-6);

  //===================================================================================
  //===================================================================================
  //===================================================================================

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
  std::vector<double> viconsigmas_default = {1e-4, 1e-4, 1e-4, 1e-5, 1e-5, 1e-5};
  nh.param<std::vector<double>>("vicon_sigmas", viconsigmas, viconsigmas_default);
  R_q(0, 0) = std::pow(viconsigmas.at(0), 2);
  R_q(1, 1) = std::pow(viconsigmas.at(1), 2);
  R_q(2, 2) = std::pow(viconsigmas.at(2), 2);
  R_p(0, 0) = std::pow(viconsigmas.at(3), 2);
  R_p(1, 1) = std::pow(viconsigmas.at(4), 2);
  R_p(2, 2) = std::pow(viconsigmas.at(5), 2);

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Our data storage objects
  std::shared_ptr<Propagator> propagator = std::make_shared<Propagator>(sigma_w, sigma_wb, sigma_a, sigma_ab);
  std::shared_ptr<Interpolator> interpolator = std::make_shared<Interpolator>();

  // Counts on how many measurements we have
  int ct_imu = 0;
  int ct_vic = 0;
  double start_time = -1;
  double end_time = -1;

  // Check if we have any interval with long gap of no vicon measurements
  // We can still try to process, but if it is too long, then the IMU can drift
  // Thus just warn the user that there might be an issue!
  double last_vicon_time = -1;
  double max_vicon_lost_time = 1.0; // seconds
  auto warn_amount_vicon_rate = [&](double timestamp, double timestamp_last) {
    double vicon_dt = timestamp - timestamp_last;
    if (last_vicon_time == -1 || vicon_dt < max_vicon_lost_time)
      return;
    double dist_from_start = timestamp_last - time_init.toSec();
    ROS_WARN("over %.2f seconds of no vicon!! (starting %.2f sec into bag)", vicon_dt, dist_from_start);
  };

  // Step through the rosbag
  ROS_INFO("load custom data into memory...");
  for (const rosbag::MessageInstance &m : view) {

    // If ros is wants us to stop, break out
    if (!ros::ok())
      break;

    // Handle IMU messages
    sensor_msgs::Imu::ConstPtr s0 = m.instantiate<sensor_msgs::Imu>();
    if (s0 != nullptr && m.getTopic() == topic_imu) {
      Eigen::Vector3d wm, am;
      wm << s0->angular_velocity.x, s0->angular_velocity.y, s0->angular_velocity.z;
      am << s0->linear_acceleration.x, s0->linear_acceleration.y, s0->linear_acceleration.z;
      propagator->feed_imu(s0->header.stamp.toSec(), wm, am);
      ct_imu++;
      continue;
    }

    // Handle VICON messages
    nav_msgs::Odometry::ConstPtr s2 = m.instantiate<nav_msgs::Odometry>();
    if (s2 != nullptr && m.getTopic() == topic_vicon) {
      // load orientation and position of the vicon
      Eigen::Vector4d q;
      Eigen::Vector3d p;
      q << s2->pose.pose.orientation.x, s2->pose.pose.orientation.y, s2->pose.pose.orientation.z, s2->pose.pose.orientation.w;
      p << s2->pose.pose.position.x, s2->pose.pose.position.y, s2->pose.pose.position.z;
      // load the covariance of the pose (order=x,y,z,rx,ry,rz) stored row-major
      Eigen::Matrix<double, 6, 6> pose_cov;
      for (size_t c = 0; c < 6; c++) {
        for (size_t r = 0; r < 6; r++) {
          pose_cov(r, c) = s2->pose.covariance[6 * c + r];
        }
      }
      // Overwrite if using manual sigmas
      if (use_manual_sigmas) {
        pose_cov = Eigen::Matrix<double, 6, 6>::Zero();
        pose_cov.block(3, 3, 3, 3) = R_q;
        pose_cov.block(0, 0, 3, 3) = R_p;
      }
      // Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>> pose_cov(s2->pose.covariance.begin(),36,1);
      //  feed it!
      interpolator->feed_pose(s2->header.stamp.toSec(), q, p, pose_cov.block(3, 3, 3, 3), pose_cov.block(0, 0, 3, 3));
      ct_vic++;
      // update timestamps
      if (start_time == -1) {
        start_time = s2->header.stamp.toSec();
      }
      if (start_time != -1) {
        end_time = s2->header.stamp.toSec();
      }
      warn_amount_vicon_rate(s2->header.stamp.toSec(), last_vicon_time);
      last_vicon_time = s2->header.stamp.toSec();
      continue;
    }

    // Handle VICON messages
    geometry_msgs::TransformStamped::ConstPtr s3 = m.instantiate<geometry_msgs::TransformStamped>();
    if (s3 != nullptr && m.getTopic() == topic_vicon) {
      // load orientation and position of the vicon
      Eigen::Vector4d q;
      Eigen::Vector3d p;
      q << s3->transform.rotation.x, s3->transform.rotation.y, s3->transform.rotation.z, s3->transform.rotation.w;
      p << s3->transform.translation.x, s3->transform.translation.y, s3->transform.translation.z;
      // feed it!
      interpolator->feed_pose(s3->header.stamp.toSec(), q, p, R_q, R_p);
      ct_vic++;
      // update timestamps
      if (start_time == -1) {
        start_time = s3->header.stamp.toSec();
      }
      if (start_time != -1) {
        end_time = s3->header.stamp.toSec();
      }
      warn_amount_vicon_rate(s3->header.stamp.toSec(), last_vicon_time);
      last_vicon_time = s3->header.stamp.toSec();
      continue;
    }

    // Handle VICON messages
    geometry_msgs::PoseStamped::ConstPtr s4 = m.instantiate<geometry_msgs::PoseStamped>();
    if (s4 != nullptr && m.getTopic() == topic_vicon) {
      // load orientation and position of the vicon
      Eigen::Vector4d q;
      Eigen::Vector3d p;
      q << s4->pose.orientation.x, s4->pose.orientation.y, s4->pose.orientation.z, s4->pose.orientation.w;
      p << s4->pose.position.x, s4->pose.position.y, s4->pose.position.z;
      // feed it!
      interpolator->feed_pose(s4->header.stamp.toSec(), q, p, R_q, R_p);
      ct_vic++;
      // update timestamps
      if (start_time == -1) {
        start_time = s4->header.stamp.toSec();
      }
      if (start_time != -1) {
        end_time = s4->header.stamp.toSec();
      }
      warn_amount_vicon_rate(s4->header.stamp.toSec(), last_vicon_time);
      last_vicon_time = s4->header.stamp.toSec();
      continue;
    }
  }
  auto rT3 = boost::posix_time::microsec_clock::local_time();
  ROS_INFO("\u001b[34m[TIME]: %.4f to preprocess\u001b[0m", (rT3 - rT2).total_microseconds() * 1e-6);

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

  // Check to make sure we have data to optimize
  if (ct_imu == 0 || ct_cam == 0 || ct_vic == 0) {
    ROS_ERROR("Not enough data to optimize with!");
    ros::shutdown();
    return EXIT_FAILURE;
  }

  // Create the graph problem, and solve it
  ViconGraphSolver solver(nh, propagator, interpolator, timestamp_cameras);
  solver.build_and_solve();

  // Visualize onto ROS
  solver.visualize();

  // Finally, save to file all the information
  if (save_to_file) {
    solver.write_to_file(path_states, path_info);
  }

  // Done!
  return EXIT_SUCCESS;
}

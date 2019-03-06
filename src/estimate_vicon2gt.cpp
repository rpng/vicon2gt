/**
 * MIT License
 * Copyright (c) 2018 Patrick Geneva @ University of Delaware (Robot Perception & Navigation Group)
 * Copyright (c) 2018 Kevin Eckenhoff @ University of Delaware (Robot Perception & Navigation Group)
 * Copyright (c) 2018 Guoquan Huang @ University of Delaware (Robot Perception & Navigation Group)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */




#include <cmath>
#include <vector>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>


#include "meas/Propagator.h"
#include "meas/Interpolator.h"
#include "solver/ViconGraphSolver.h"


int main(int argc, char** argv)
{

    // Start up
    ros::init(argc, argv, "estimate_vicon2gt");
    ros::NodeHandle nh("~");

    // Load the imu, camera, and vicon topics
    std::string topic_imu, topic_cam, topic_vicon;
    nh.param<std::string>("topic_imu", topic_imu, "/imu0");
    nh.param<std::string>("topic_cam", topic_cam, "/cam0/image_raw");
    nh.param<std::string>("topic_vicon", topic_vicon, "/vicon/ironsides/odom");

    // Load the bag path
    bool save2file;
    std::string path_to_bag, path_states, path_info;
    nh.param<std::string>("path_bag", path_to_bag, "/home/patrick/datasets/ARL/ironsides_tracking/2019-02-22-12-03-15.bag");
    nh.param<std::string>("stats_path_states", path_states, "gt_states.csv");
    nh.param<std::string>("stats_path_info", path_info, "vicon2gt_info.txt");
    nh.param<bool>("save2file", save2file, false);
    ROS_INFO("rosbag information...");
    ROS_INFO("    - bag path: %s", path_to_bag.c_str());
    ROS_INFO("    - state path: %s", path_states.c_str());
    ROS_INFO("    - info path: %s", path_info.c_str());
    ROS_INFO("    - save to file: %d", (int)save2file);

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
    view_full.addQuery(bag);
    ros::Time time_init = view_full.getBeginTime();
    time_init += ros::Duration(bag_start);
    ros::Time time_finish = (bag_durr < 0)? view_full.getEndTime() : time_init + ros::Duration(bag_durr);
    ROS_INFO("loading rosbag...");
    ROS_INFO("    - time start = %.6f", time_init.toSec());
    ROS_INFO("    - time end   = %.6f", time_finish.toSec());
    ROS_INFO("    - duration   = %.2f (secs)", time_finish.toSec()-time_init.toSec());
    view.addQuery(bag, time_init, time_finish);

    // Check to make sure we have data to play
    if (view.size() == 0) {
        ROS_ERROR("No messages to play on specified topics.  Exiting.");
        ROS_ERROR("IMU TOPIC: %s",topic_imu.c_str());
        ROS_ERROR("CAM TOPIC: %s",topic_cam.c_str());
        ROS_ERROR("VIC TOPIC: %s",topic_vicon.c_str());
        ros::shutdown();
        return EXIT_FAILURE;
    }


    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Our IMU noise values
    double sigma_w,sigma_wb,sigma_a,sigma_ab;
    nh.param<double>("gyroscope_noise_density", sigma_w, 1.6968e-04);
    nh.param<double>("accelerometer_noise_density", sigma_a, 2.0000e-3);
    nh.param<double>("gyroscope_random_walk", sigma_wb, 1.9393e-05);
    nh.param<double>("accelerometer_random_walk", sigma_ab, 3.0000e-03);

    // Vicon sigmas (used if we don't have odometry messages)
    Eigen::Matrix<double,3,3> R_q = Eigen::Matrix<double,3,3>::Zero();
    Eigen::Matrix<double,3,3> R_p = Eigen::Matrix<double,3,3>::Zero();
    std::vector<double> viconsigmas;
    std::vector<double> viconsigmas_default = {1e-4,1e-4,1e-4,1e-5,1e-5,1e-5};
    nh.param<std::vector<double>>("vicon_sigmas", viconsigmas, viconsigmas_default);
    R_q(0,0) = std::pow(viconsigmas.at(0),2);
    R_q(1,1) = std::pow(viconsigmas.at(1),2);
    R_q(2,2) = std::pow(viconsigmas.at(2),2);
    R_p(0,0) = std::pow(viconsigmas.at(3),2);
    R_p(1,1) = std::pow(viconsigmas.at(4),2);
    R_p(2,2) = std::pow(viconsigmas.at(5),2);


    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Our data storage objects
    Propagator* propagator = new Propagator(sigma_w,sigma_wb,sigma_a,sigma_ab);
    Interpolator* interpolator = new Interpolator();
    std::vector<double> timestamp_cameras;

    // Counts on how many measurements we have
    int ct_imu = 0;
    int ct_cam = 0;
    int ct_vic = 0;

    // Step through the rosbag
    for (const rosbag::MessageInstance& m : view) {

        // If ros is wants us to stop, break out
        if (!ros::ok())
            break;

        // Handle IMU messages
        sensor_msgs::Imu::ConstPtr s0 = m.instantiate<sensor_msgs::Imu>();
        if (s0 != NULL && m.getTopic() == topic_imu) {
            Eigen::Matrix<double,3,1> wm, am;
            wm << s0->angular_velocity.x, s0->angular_velocity.y, s0->angular_velocity.z;
            am << s0->linear_acceleration.x, s0->linear_acceleration.y, s0->linear_acceleration.z;
            propagator->feed_imu(s0->header.stamp.toSec(),wm,am);
            ct_imu++;
        }

        // Handle CAMEREA messages
        sensor_msgs::Image::ConstPtr s1 = m.instantiate<sensor_msgs::Image>();
        if (s1 != NULL && m.getTopic() == topic_cam) {
            timestamp_cameras.push_back(s1->header.stamp.toSec());
            ct_cam++;
        }

        // Handle VICON messages
        nav_msgs::Odometry::ConstPtr s2 = m.instantiate<nav_msgs::Odometry>();
        if (s2 != NULL && m.getTopic() == topic_vicon) {
            // load orientation and position of the vicon
            Eigen::Matrix<double,4,1> q;
            Eigen::Matrix<double,3,1> p;
            q << s2->pose.pose.orientation.x,s2->pose.pose.orientation.y,s2->pose.pose.orientation.z,s2->pose.pose.orientation.w;
            p << s2->pose.pose.position.x,s2->pose.pose.position.y,s2->pose.pose.position.z;
            // load the covariance of the pose (order=x,y,z,rx,ry,rz) stored row-major
            Eigen::Matrix<double,6,6> pose_cov;
            for(size_t c=0;c<6;c++) {
                for(size_t r=0;r<6;r++) {
                    pose_cov(r,c) = s2->pose.covariance[6*c+r];
                }
            }
            //Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>> pose_cov(s2->pose.covariance.begin(),36,1);
            // feed it!
            interpolator->feed_pose(s2->header.stamp.toSec(),q,p,pose_cov.block(3,3,3,3),pose_cov.block(0,0,3,3));
            ct_vic++;
        }

        // Handle VICON messages
        geometry_msgs::TransformStamped::ConstPtr s3 = m.instantiate<geometry_msgs::TransformStamped>();
        if (s3 != NULL && m.getTopic() == topic_vicon) {
            // load orientation and position of the vicon
            Eigen::Matrix<double,4,1> q;
            Eigen::Matrix<double,3,1> p;
            q << s3->transform.rotation.x,s3->transform.rotation.y,s3->transform.rotation.z,s3->transform.rotation.w;
            p << s3->transform.translation.x,s3->transform.translation.y,s3->transform.translation.z;
            // feed it!
            interpolator->feed_pose(s3->header.stamp.toSec(),q,p,R_q,R_p);
            ct_vic++;
        }

        // Handle VICON messages
        geometry_msgs::PoseStamped::ConstPtr s4 = m.instantiate<geometry_msgs::PoseStamped>();
        if (s4 != NULL && m.getTopic() == topic_vicon) {
            // load orientation and position of the vicon
            Eigen::Matrix<double,4,1> q;
            Eigen::Matrix<double,3,1> p;
            q << s4->pose.orientation.x,s4->pose.orientation.y,s4->pose.orientation.z,s4->pose.orientation.w;
            p << s4->pose.position.x,s4->pose.position.y,s4->pose.position.z;
            // feed it!
            interpolator->feed_pose(s4->header.stamp.toSec(),q,p,R_q,R_p);
            ct_vic++;
        }


    }

    // Print out how many we have loaded
    ROS_INFO("done loading the rosbag...");
    ROS_INFO("    - number imu   = %d",ct_imu);
    ROS_INFO("    - number cam   = %d",ct_cam);
    ROS_INFO("    - number vicon = %d",ct_vic);

    // Create the graph problem, and solve it
    ViconGraphSolver solver(nh,propagator,interpolator,timestamp_cameras);
    solver.build_and_solve();


    // Finally, save to file all the information
    if(save2file) {
        solver.write_to_file(path_states,path_info);
    }


    // Done!
    return EXIT_SUCCESS;
}





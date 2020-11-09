/**
 * MIT License
 * Copyright (c) 2020 Patrick Geneva @ University of Delaware (Robot Perception & Navigation Group)
 * Copyright (c) 2020 Guoquan Huang @ University of Delaware (Robot Perception & Navigation Group)
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
#include <memory>
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
#include "sim/Simulator.h"
#include "utils/stats.h"


int main(int argc, char** argv)
{

    // Start up
    ros::init(argc, argv, "run_simulation");
    ros::NodeHandle nh("~");

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
    double sigma_w,sigma_wb,sigma_a,sigma_ab;
    nh.param<double>("gyroscope_noise_density", sigma_w, 1.6968e-04);
    nh.param<double>("accelerometer_noise_density", sigma_a, 2.0000e-3);
    nh.param<double>("gyroscope_random_walk", sigma_wb, 1.9393e-05);
    nh.param<double>("accelerometer_random_walk", sigma_ab, 3.0000e-03);

    // Vicon sigmas (used if we don't have odometry messages)
    Eigen::Matrix<double,3,3> R_q = Eigen::Matrix<double,3,3>::Zero();
    Eigen::Matrix<double,3,3> R_p = Eigen::Matrix<double,3,3>::Zero();
    std::vector<double> viconsigmas;
    std::vector<double> viconsigmas_default = {1e-3,1e-3,1e-3,1e-2,1e-2,1e-2};
    nh.param<std::vector<double>>("vicon_sigmas", viconsigmas, viconsigmas_default);
    R_q(0,0) = std::pow(viconsigmas.at(0),2);
    R_q(1,1) = std::pow(viconsigmas.at(1),2);
    R_q(2,2) = std::pow(viconsigmas.at(2),2);
    R_p(0,0) = std::pow(viconsigmas.at(3),2);
    R_p(1,1) = std::pow(viconsigmas.at(4),2);
    R_p(2,2) = std::pow(viconsigmas.at(5),2);
    params.sigma_vicon_pose << viconsigmas_default.at(0), viconsigmas_default.at(1), viconsigmas_default.at(2),
                    viconsigmas_default.at(3), viconsigmas_default.at(4), viconsigmas_default.at(5);

    // Load gravity in vicon frame
    std::vector<double> vec_gravity;
    std::vector<double> vec_gravity_default = {0.0,0.0,9.8};
    nh.param<std::vector<double>>("grav_inV", vec_gravity, vec_gravity_default);
    params.gravity << vec_gravity.at(0),vec_gravity.at(1),vec_gravity.at(2);

    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Our simulator
    std::shared_ptr<Simulator> sim = std::make_shared<Simulator>(params);

    // Our data storage objects
    std::shared_ptr<Propagator> propagator = std::make_shared<Propagator>(sigma_w,sigma_wb,sigma_a,sigma_ab);
    std::shared_ptr<Interpolator> interpolator = std::make_shared<Interpolator>();
    std::vector<double> timestamp_cameras;

    // Counts on how many measurements we have
    int ct_imu = 0;
    int ct_cam = 0;
    int ct_vic = 0;

    // Step through the rosbag
    while(sim->ok() && ros::ok()) {

        // IMU: get the next simulated IMU measurement if we have it
        double time_imu;
        Eigen::Vector3d wm, am;
        if (sim->get_next_imu(time_imu, wm, am)) {
            propagator->feed_imu(time_imu,wm,am);
            ct_imu++;
        }

        // CAM: get the next simulated camera uv measurements if we have them
        double time_cam;
        if (sim->get_next_cam(time_cam)) {
            timestamp_cameras.push_back(time_cam);
            ct_cam++;
        }

        // VICON: get the next simulated camera uv measurements if we have them
        double time_vicon;
        Eigen::Vector4d q_VtoB;
        Eigen::Vector3d p_VinB;
        if (sim->get_next_vicon(time_vicon, q_VtoB, p_VinB)) {
            interpolator->feed_pose(time_vicon,q_VtoB,p_VinB,R_q,R_p);
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

    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Get the final optimized poses
    std::vector<double> times;
    std::vector<Eigen::Matrix<double,7,1>> poses;
    solver.get_imu_poses(times, poses);

    // Now compute the error compared to our true states
    Stats err_ori, err_pos;
    for(size_t i=0; i<times.size(); i++) {

        // get the states
        // GT: [time(sec),q_GtoI,p_IinG,v_IinG,b_gyro,b_accel]
        // EST: [q_GtoI,p_IinG]
        Eigen::Matrix<double,17,1> gt_state;
        sim->get_state(times.at(i), gt_state);
        Eigen::Matrix<double,7,1> est_state = poses.at(i);

        // compute error
        double ori = 2*(quat_multiply(
                est_state.block(0,0,4,1),
                Inv(gt_state.block(1,0,4,1))
        )).block(0,0,3,1).norm();
        double pose = (est_state.block(4,0,3,1)-gt_state.block(5,0,3,1)).norm();

        // Append to the history
        //cout << i << " " << 180.0/M_PI*ori << " " << pose << endl;
        //cout << gt_state.block(1,0,7,1).transpose() << endl;
        //cout << est_state.transpose() << endl;
        err_ori.timestamps.push_back(times.at(i));
        err_ori.values.push_back(180.0/M_PI*ori);
        err_pos.timestamps.push_back(times.at(i));
        err_pos.values.push_back(pose);

    }

    // Calculate and print trajectory error
    err_ori.calculate();
    err_pos.calculate();
    printf(REDPURPLE "======================================\n");
    printf(REDPURPLE "Trajectory Errors (deg,m)\n");
    printf(REDPURPLE "======================================\n");
    printf(REDPURPLE "rmse_ori = %.5f | rmse_pos = %.5f\n",err_ori.rmse,err_pos.rmse);
    printf(REDPURPLE "mean_ori = %.5f | mean_pos = %.5f\n",err_ori.mean,err_pos.mean);
    printf(REDPURPLE "min_ori  = %.5f | min_pos  = %.5f\n",err_ori.min,err_pos.min);
    printf(REDPURPLE "max_ori  = %.5f | max_pos  = %.5f\n",err_ori.max,err_pos.max);
    printf(REDPURPLE "std_ori  = %.5f | std_pos  = %.5f\n\n",err_ori.std,err_pos.std);


    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Get converged calibration
    double toff;
    Eigen::Matrix3d R_BtoI;
    Eigen::Vector3d p_BinI, grav_inV;
    solver.get_calibration(toff, R_BtoI, p_BinI, grav_inV);
    Eigen::Vector4d q_BtoI = rot_2_quat(R_BtoI);
    Eigen::Vector4d gt_q_BtoI = rot_2_quat(params.R_BtoI);

    printf(REDPURPLE "======================================\n");
    printf(REDPURPLE "Converged Calib vs Groundtruth\n");
    printf(REDPURPLE "======================================\n");
    printf(REDPURPLE "GT  toff: %.5f\n", params.viconimu_dt);
    printf(REDPURPLE "EST toff: %.5f\n\n", toff);
    printf(REDPURPLE "GT  grav_inV: %.3f, %.3f, %.3f\n", params.gravity(0), params.gravity(1), params.gravity(2));
    printf(REDPURPLE "EST grav_inV: %.3f, %.3f, %.3f\n\n", grav_inV(0), grav_inV(1), grav_inV(2));
    printf(REDPURPLE "GT  q_BtoI: %.3f, %.3f, %.3f, %.3f\n", gt_q_BtoI(0), gt_q_BtoI(1), gt_q_BtoI(2), gt_q_BtoI(3));
    printf(REDPURPLE "EST q_BtoI: %.3f, %.3f, %.3f, %.3f\n\n", q_BtoI(0), q_BtoI(1), q_BtoI(2), q_BtoI(3));
    printf(REDPURPLE "GT  p_BinI: %.3f, %.3f, %.3f\n", params.p_BinI(0), params.p_BinI(1), params.p_BinI(2));
    printf(REDPURPLE "EST p_BinI: %.3f, %.3f, %.3f\n\n", p_BinI(0), p_BinI(1), p_BinI(2));



    // Done!
    return EXIT_SUCCESS;
}

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
#ifndef SIMULATORPARAMS_H
#define SIMULATORPARAMS_H

#include <string>
#include <vector>
#include <iostream>
#include <Eigen/Eigen>


#include "utils/colors.h"
#include "utils/quat_ops.h"


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
    Eigen::Matrix<double,6,1> sigma_vicon_pose = (Eigen::Matrix<double,6,1>() << 1e-3,1e-3,1e-3,1e-2,1e-2,1e-2).finished();

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

    /// Gravity in the global frame (i.e. should be [0, 0, 9.81] typically)
    Eigen::Vector3d gravity = {0.0, 0.0, 9.81};

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
        printf(CYAN "\t- gravity: %.3f, %.3f, %.3f\n", gravity(0), gravity(1), gravity(2));
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

    /// This should be incremented for each run in the Monte-Carlo simulation to generate the same true measurements, but different noise values.
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


#endif //SIMULATORPARAMS_H
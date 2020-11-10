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


#ifndef VICONGRAPHSOLVER_H
#define VICONGRAPHSOLVER_H


#include <vector>
#include <fstream>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/PriorFactor.h>

#include "cpi/CpiV1.h"
#include "gtsam/GtsamConfig.h"
#include "gtsam/JPLNavState.h"
#include "gtsam/JPLQuaternion.h"
#include "gtsam/MeasBased_ViconPoseTimeoffsetFactor.h"
#include "gtsam/ImuFactorCPIv1.h"
#include "gtsam/MagnitudePrior.h"
#include "meas/Propagator.h"
#include "meas/Interpolator.h"
#include "utils/quat_ops.h"


#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>


using namespace std;
using namespace gtsam;


using gtsam::symbol_shorthand::X; // X: our JPL states
using gtsam::symbol_shorthand::C; // C: calibration (c(0)=rot, c(1)=pos)
using gtsam::symbol_shorthand::G; // G: global gravity in the vicon frame
using gtsam::symbol_shorthand::T; // T: time offset between vicon and imu sensors



class ViconGraphSolver
{

public:

    /**
     * @brief Default constructor for the solver
     * @param nh ROS node handler we will load parameters from
     * @param propagator Propagator with all IMU measurements inside
     * @param interpolator Interpolator with all vicon poses inside
     * @param timestamp_cameras Timestamps we are interested in estimating
     */
    ViconGraphSolver(ros::NodeHandle& nh, std::shared_ptr<Propagator> propagator,
                     std::shared_ptr<Interpolator> interpolator, std::vector<double> timestamp_cameras);


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
     * @param poses Poses in quaternion position ordering
     */
    void get_imu_poses(std::vector<double> &times, std::vector<Eigen::Matrix<double,7,1>> &poses);

    /**
     * @brief Gets other calibration parameters we estimate online
     * @param toff Time offset between vicon and IMU
     * @param R Rotation between vicon and IMU
     * @param p Position between vicon and IMU
     * @param g Gravity in the vicon frame
     */
    void get_calibration(double &toff, Eigen::Matrix3d &R, Eigen::Vector3d &p, Eigen::Vector3d &g);


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
    ros::Publisher pub_pathimu, pub_pathvicon;

    // Measurement data from the rosbag
    std::shared_ptr<Propagator> propagator;
    std::shared_ptr<Interpolator> interpolator;
    std::vector<double> timestamp_cameras;

    // Initial estimates of our variables
    Eigen::Matrix<double,3,1> init_grav_inV;
    Eigen::Matrix<double,3,3> init_R_BtoI;
    Eigen::Matrix<double,3,1> init_p_BinI;
    double init_toff_imu_to_vicon;

    // Master non-linear GTSAM graph, all created factors
    // Also have all nodes in the graph
    gtsam::NonlinearFactorGraph* graph;
    gtsam::Values values;

    // Optimized values
    gtsam::Values values_result;

    // Config for what we are optimizing
    std::shared_ptr<GtsamConfig> config;

    // Map between state timestamp and their IDs
    std::map<double,size_t> map_states;

    // If we should enforce gravity magnitude
    bool enforce_grav_mag;

    // Number of times we will loop and relinearize the measurements
    int num_loop_relin;

};




#endif /* VICONGRAPHSOLVER_H */

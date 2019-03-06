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




#include "ViconGraphSolver.h"



/**
 * Default constructor.
 * Will load all needed configuration variables from the launch file and construct the graph.
 */
ViconGraphSolver::ViconGraphSolver(ros::NodeHandle& nh, Propagator* propagator,
                                   Interpolator* interpolator, std::vector<double> timestamp_cameras) {

    // save measurement data
    this->propagator = propagator;
    this->interpolator = interpolator;
    this->timestamp_cameras = timestamp_cameras;

    // Initalize our graphs
    this->graph = new gtsam::NonlinearFactorGraph();
    this->graph_new = new gtsam::NonlinearFactorGraph();

    // Load gravity in vicon frame
    std::vector<double> vec_gravity;
    std::vector<double> vec_gravity_default = {0.0,0.0,9.8};
    nh.param<std::vector<double>>("grav_inV", vec_gravity, vec_gravity_default);
    init_grav_inV << vec_gravity.at(0),vec_gravity.at(1),vec_gravity.at(2);

    // Load transform between vicon body frame to the IMU
    std::vector<double> R_BtoI;
    std::vector<double> R_BtoI_default = {1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0};
    nh.param<std::vector<double>>("R_BtoI", R_BtoI, R_BtoI_default);
    init_R_BtoI << R_BtoI.at(0),R_BtoI.at(1),R_BtoI.at(2),R_BtoI.at(3),R_BtoI.at(4),R_BtoI.at(5),R_BtoI.at(6),R_BtoI.at(7),R_BtoI.at(8);

    std::vector<double> p_BinI;
    std::vector<double> p_BinI_default = {0.0,0.0,0.0};
    nh.param<std::vector<double>>("p_BinI", p_BinI, p_BinI_default);
    init_p_BinI << p_BinI.at(0),p_BinI.at(1),p_BinI.at(2);

    // Debug print to console
    cout << "init_grav_inV:" << endl << init_grav_inV << endl;
    cout << "init_R_BtoI:" << endl << init_R_BtoI << endl;
    cout << "init_p_BinI:" << endl << init_p_BinI << endl;



}




/**
 * This will first build the graph problem and then solve it
 * This function will take a while, but handles the GTSAM optimization
 */
void ViconGraphSolver::build_and_solve() {

    // Start timing
    rT1 =  boost::posix_time::microsec_clock::local_time();

    // Ensure we have enough measurements
    if(timestamp_cameras.empty()) {
        ROS_ERROR("[VICON-GRAPH]: Camera timestamp vector empty!!!!");
        ROS_ERROR("[VICON-GRAPH]: Make sure your camera topic is correct...");
        ROS_ERROR("%s on line %d",__FILE__,__LINE__);
        std::exit(EXIT_FAILURE);
    }

    // Delete all camera measurements that occur before our IMU readings
    ROS_INFO("cleaning camera timestamps");
    auto it0 = timestamp_cameras.begin();
    while(it0 != timestamp_cameras.end()) {
        if(!propagator->has_bounding_imu(*it0)) {
            ROS_INFO("    - deleted cam time %.9f",*it0);
            it0 = timestamp_cameras.erase(it0);
        } else {
            it0++;
        }
    }

    // Ensure we have enough measurements after removing invalid
    if(timestamp_cameras.empty()) {
        ROS_ERROR("[VICON-GRAPH]: All camera timestamps where out of the range of the IMU measurements.");
        ROS_ERROR("[VICON-GRAPH]: Make sure your camera and imu topics are correct...");
        ROS_ERROR("%s on line %d",__FILE__,__LINE__);
        std::exit(EXIT_FAILURE);
    }


    // Create map of the state timestamps to their IDs
    for(size_t i=0; i<timestamp_cameras.size(); i++) {
        map_states.insert({timestamp_cameras.at(i),i});
    }


    // Create gravity and calibration nodes and insert them
    values.insert(C(0), Rot3(init_R_BtoI));
    values_new.insert(C(0), Rot3(init_R_BtoI));
    values.insert(C(1), Vector3(init_p_BinI));
    values_new.insert(C(1), Vector3(init_p_BinI));
    values.insert(G(0), Vector3(init_grav_inV));
    values_new.insert(G(0), Vector3(init_grav_inV));


    // Loop through each camera time and construct the graph
    auto it1 = timestamp_cameras.begin();
    while(it1 != timestamp_cameras.end()) {

        // If ros is wants us to stop, break out
        if (!ros::ok())
            break;

        // Current image time
        double timestamp = *it1;

        // First get the vicon pose at the current time
        Eigen::Matrix<double,4,1> q_VtoB;
        Eigen::Matrix<double,3,1> p_BinV;
        Eigen::Matrix<double,6,6> R_vicon;
        bool has_vicon = interpolator->get_pose(timestamp,q_VtoB,p_BinV,R_vicon);

        // Skip if we don't have a vicon measurement for this pose
        if(!has_vicon) {
            ROS_INFO("skipping vicon time %.9f (no vicon pose found)",timestamp);
            it1 = timestamp_cameras.erase(it1);
            continue;
        }

        // Check if we can do the inverse
        if(std::isnan(R_vicon.norm()) || std::isnan(R_vicon.inverse().norm())) {
            ROS_INFO("skipping vicon time %.9f (R.norm = %.3f | Rinv.norm = %.3f)",timestamp,R_vicon.norm(),R_vicon.inverse().norm());
            it1 = timestamp_cameras.erase(it1);
            continue;
        }

        // Now initialize the current pose of the IMU
        Eigen::Matrix<double,4,1> q_VtoI = quat_multiply(rot_2_quat(init_R_BtoI),q_VtoB);
        Eigen::Matrix<double,3,1> bg = Eigen::Matrix<double,3,1>::Zero();
        Eigen::Matrix<double,3,1> v_IinV = Eigen::Matrix<double,3,1>::Zero();
        Eigen::Matrix<double,3,1> ba = Eigen::Matrix<double,3,1>::Zero();
        Eigen::Matrix<double,3,1> p_IinV = p_BinV - quat_2_Rot(Inv(q_VtoB))*init_R_BtoI.transpose()*init_p_BinI;
        JPLNavState imu_state(q_VtoI, bg, v_IinV, ba, p_IinV);
        values.insert(X(map_states[timestamp]), imu_state);
        values_new.insert(X(map_states[timestamp]), imu_state);

        // Add the vicon measurement to this pose
        ViconPoseFactor factor_vicon(X(map_states[timestamp]),C(0),C(1),R_vicon,q_VtoB,p_BinV);
        graph->add(factor_vicon);
        graph_new->add(factor_vicon);

        // Skip the first ever pose
        if(it1 == timestamp_cameras.begin()) {
            it1++;
            continue;
        }

        // Now add preintegration between this state and the next
        // We do a silly hack since inside of the propagator we create the preintegrator
        // So we just randomly assign noises here which will be overwritten in the propagator
        double time0 = *(it1-1);
        double time1 = *(it1);
        CpiV1 preint(0,0,0,0,false);
        bool has_imu = propagator->propagate(time0,time1,bg,ba,preint);
        assert(has_imu);
        assert(preint.DT==(time1-time0));

        //cout << "dt = " << preint.DT << " | dt_times = " << time1-time0 << endl;
        //cout << "q_k2tau = " << preint.q_k2tau.transpose() << endl;
        //cout << "alpha_tau = " << preint.alpha_tau.transpose() << endl;
        //cout << "beta_tau = " << preint.beta_tau.transpose() << endl;

        // Check if we can do the inverse
        if(std::isnan(preint.P_meas.norm()) || std::isnan(preint.P_meas.inverse().norm())) {
            ROS_ERROR("R_imu is NAN | R.norm = %.3f | Rinv.norm = %.3f",preint.P_meas.norm(),preint.P_meas.inverse().norm());
            ROS_ERROR("THIS SHOULD NEVER HAPPEN!@#!@#!@#!@#!#@");
        }

        // Now create the IMU factor
        ImuFactorCPIv1 factor_imu(X(map_states[time0]),X(map_states[time1]),G(0),preint.P_meas,preint.DT,preint.alpha_tau,preint.beta_tau,
                                  preint.q_k2tau,preint.b_a_lin,preint.b_w_lin,preint.J_q,preint.J_b,preint.J_a,preint.H_b,preint.H_a);
        graph->add(factor_imu);
        graph_new->add(factor_imu);

        // Finally, move forward in time!
        it1++;

    }
    rT2 =  boost::posix_time::microsec_clock::local_time();


    // Debug
    ROS_INFO("[VICON-GRAPH]: graph factors - %d", (int) graph->nrFactors());
    ROS_INFO("[VICON-GRAPH]: graph nodes - %d", (int) graph->keys().size());

    // Setup the optimizer
    LevenbergMarquardtParams config;
    config.verbosity = NonlinearOptimizerParams::Verbosity::TERMINATION;
    config.verbosityLM = LevenbergMarquardtParams::VerbosityLM::SUMMARY;
    config.setRelativeErrorTol(1e-30);
    LevenbergMarquardtOptimizer optimizer(*graph, values, config);

    // Perform the optimization
    ROS_INFO("[VICON-GRAPH]: begin optimization");
    gtsam::Values result_values = optimizer.optimize();
    ROS_INFO("[VICON-GRAPH]: done optimization (%d iterations)!", (int) optimizer.iterations());
    rT3 = boost::posix_time::microsec_clock::local_time();


    // Now print timing statistics
    ROS_INFO("\u001b[34m[TIME]: %.4f to build\u001b[0m",(rT2-rT1).total_microseconds() * 1e-6);
    ROS_INFO("\u001b[34m[TIME]: %.4f to optimize\u001b[0m",(rT3-rT2).total_microseconds() * 1e-6);
    ROS_INFO("\u001b[34m[TIME]: %.4f total\u001b[0m",(rT3-rT1).total_microseconds() * 1e-6);


    // Debug print results...
    cout << endl << "======================================" << endl;
    cout << "state_0: " << endl << result_values.at<JPLNavState>(X(map_states[timestamp_cameras.at(0)])) << endl;
    cout << "state_N: " << endl << result_values.at<JPLNavState>(X(map_states[timestamp_cameras.at(timestamp_cameras.size()-1)])) << endl;
    cout << "R_BtoI: " << result_values.at<Rot3>(C(0)) << endl;
    cout << "p_BinI: " << endl << result_values.at<Vector3>(C(1)) << endl << endl;
    cout << "gravity: " << endl << result_values.at<Vector3>(G(0)) << endl << endl;
    cout << "gravity norm: " << endl << result_values.at<Vector3>(G(0)).norm() << endl;
    cout << "======================================" << endl;


}










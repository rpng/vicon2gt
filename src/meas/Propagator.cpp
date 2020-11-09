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


#include "Propagator.h"



/**
 * This should append incoming IMU messages to our stored data array
 */
void Propagator::feed_imu(double timestamp, Eigen::Matrix<double,3,1> wm, Eigen::Matrix<double,3,1> am) {

    // Create our imu data object
    IMUDATA data;
    data.timestamp = timestamp;
    data.wm = wm;
    data.am = am;

    // Append it to our vector
    imu_data.emplace_back(data);

}



/**
 * Given two timestamps and the bias linerization points (can be taken to be zero at the start)
 * This will propagate between the specified timestamps with the IMU messages we have currently stored
 */
bool Propagator::propagate(double time0, double time1, Eigen::Matrix<double,3,1> bg_lin, Eigen::Matrix<double,3,1> ba_lin, CpiV1& integration) {


    // First lets construct an IMU vector of measurements we need
    vector<IMUDATA> prop_data;

    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Ensure we have some measurements in the first place!
    if(imu_data.empty()) {
        ROS_ERROR("[TIME-SYNC]: There are no IMU measurements!!!!!");
        ROS_ERROR("[TIME-SYNC]: Make sure you feed IMU readings into the propagator...");
        ROS_ERROR("%s on line %d",__FILE__,__LINE__);
        std::exit(EXIT_FAILURE);
    }

    // Loop through and find all the needed measurements to propagate with
    // Note we split measurements based on the given state time, and the update timestamp
    for(size_t i=0; i<imu_data.size()-1; i++) {

        // START OF THE INTEGRATION PERIOD
        // If the next timestamp is greater then our current state time
        // And the current is not greater then it yet...
        // Then we should "split" our current IMU measurement
        if(imu_data.at(i+1).timestamp > time0 && imu_data.at(i).timestamp <= time0) {
            IMUDATA data = interpolate_data(imu_data.at(i),imu_data.at(i+1),time0);
            prop_data.push_back(data);
            //ROS_INFO("propagation #%d = CASE 1 = %.3f => %.3f",(int)i,data.timestamp-prop_data.at(0).timestamp,time0-prop_data.at(0).timestamp);
            continue;
        }

        // MIDDLE OF INTEGRATION PERIOD
        // If our imu measurement is right in the middle of our propagation period
        // Then we should just append the whole measurement time to our propagation vector
        if(imu_data.at(i).timestamp >= time0 && imu_data.at(i+1).timestamp <= time1) {
            prop_data.push_back(imu_data.at(i));
            //ROS_INFO("propagation #%d = CASE 2 = %.3f",(int)i,imu_data.at(i).timestamp-prop_data.at(0).timestamp);
            continue;
        }

        // END OF THE INTEGRATION PERIOD
        // If the current timestamp is greater then our update time
        // We should just "split" the NEXT IMU measurement to the update time,
        // NOTE: we add the current time, and then the time at the end of the inverval (so we can get a dt)
        // NOTE: we also break out of this loop, as this is the last IMU measurement we need!
        if(imu_data.at(i+1).timestamp > time1) {
            prop_data.push_back(imu_data.at(i));
            // If the added IMU message doesn't end exactly at the camera time
            // Then we need to add another one that is right at the ending time
            if(prop_data.at(prop_data.size()-1).timestamp != time1) {
                IMUDATA data = interpolate_data(imu_data.at(i), imu_data.at(i + 1),time1);
                prop_data.push_back(data);
                //ROS_INFO("propagation #%d = CASE 3 = %.3f => %.3f",(int)i,data.timestamp-prop_data.at(0).timestamp,data.timestamp-prop_data.at(0).timestamp);
            }
            break;
        }

    }

    // Check that we have at least one measurement to propagate with
    if(prop_data.empty()) {
        ROS_ERROR("[TIME-SYNC]: There are not enough measurements to propagate with %d of 2",(int)prop_data.size());
        ROS_ERROR("[TIME-SYNC]: Make sure you feed IMU readings into the propagator...");
        ROS_ERROR("%s on line %d",__FILE__,__LINE__);
        std::exit(EXIT_FAILURE);
        //return false;
    }

    // If we did not reach the whole integration period (i.e., the last inertial measurement we have is smaller then the time we want to reach)
    // Then we should just "stretch" the last measurement to be the whole period
    if(imu_data.at(imu_data.size()-1).timestamp < time1) {
        IMUDATA data = interpolate_data(imu_data.at(imu_data.size()-2),imu_data.at(imu_data.size()-1),time1);
        prop_data.push_back(data);
        //ROS_INFO("propagation #%d = CASE 4 = %.3f",(int)(imu_data.size()-1),data.timestamp-prop_data.at(0).timestamp);
    }


    // Loop through and ensure we do not have an zero dt values
    // This would cause the noise covariance to be Infinity
    for (size_t i=0; i < prop_data.size()-1; i++){
        //cout << "dt - " << (prop_data.at(i+1).timestamp-prop_data.at(i).timestamp) << endl;
        if ((prop_data.at(i+1).timestamp-prop_data.at(i).timestamp) < 1e-8){
            ROS_ERROR("[TIME-SYNC]: Zero DT between %d and %d measurements (dt = %.8f)",(int)i,(int)(i+1),(prop_data.at(i+1).timestamp-prop_data.at(i).timestamp));
            prop_data.erase(prop_data.begin()+i);
            i--;
        }
    }


    // Check that we have at least one measurement to propagate with
    if(prop_data.empty()) {
        ROS_ERROR("[TIME-SYNC]: There are not enough measurements to propagate with %d of 2",(int)prop_data.size());
        ROS_ERROR("[TIME-SYNC]: Make sure you feed IMU readings into the propagator...");
        ROS_ERROR("%s on line %d",__FILE__,__LINE__);
        std::exit(EXIT_FAILURE);
        //return false;
    }

    // Debug
    //ROS_INFO("end_prop - start_prop %.3f",prop_data.at(prop_data.size()-1).timestamp-prop_data.at(0).timestamp);
    //ROS_INFO("imu_data_last - state %.3f",imu_data.at(imu_data.size()-1).timestamp-time0);
    //ROS_INFO("update_time - state %.3f",time1-time0);


    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Create our IMU measurement between the current state time, and the update time
    integration = CpiV1(sigma_w, sigma_wb, sigma_a, sigma_ab, true);

    // CPI linearization points
    integration.setLinearizationPoints(bg_lin,ba_lin);

    // Loop through all IMU messages, and use them to compute our preintegration measurement
    // Note: we do this for all three versions, but we will only use one in our graph
    for(size_t i=0; i<prop_data.size()-1; i++) {
        integration.feed_IMU(prop_data.at(i).timestamp,prop_data.at(i+1).timestamp,
                             prop_data.at(i).wm,prop_data.at(i).am,
                             prop_data.at(i+1).wm,prop_data.at(i+1).am);
    }

    // Debug messages
    //cout << "q_k2tau = " << integration->q_k2tau.transpose() << endl;
    //cout << "alpha_tau = " << integration->alpha_tau.transpose() << endl;
    //cout << "beta_tau = " << integration->beta_tau.transpose() << endl;

    // Finally return this preintegration
    return true;

}




/**
 * This function will check if we have IMU measurements before and after the given timestamp
 */
bool Propagator::has_bounding_imu(double timestamp) {

    // Ensure we have some measurements in the first place!
    if (imu_data.empty()) {
        return false;
    }

    // If we have a lower and upper bounding imu
    bool has_lower = false;
    bool has_upper = false;

    // Loop through and find bounding IMU
    for (size_t i = 0; i < imu_data.size(); i++) {
        // Check if we have a lower bound
        if (imu_data.at(i).timestamp <= timestamp) {
            has_lower = true;
        }
        // Check if we have an upper bound
        if (imu_data.at(i).timestamp >= timestamp) {
            has_upper = true;
        }
    }

    // Return if we found
    return (has_lower && has_upper);

}
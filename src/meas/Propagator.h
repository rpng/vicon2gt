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


#ifndef PROPAGATOR_H
#define PROPAGATOR_H


#include <vector>
#include <Eigen/Eigen>
#include <ros/ros.h>

#include "cpi/CpiV1.h"
#include "utils/quat_ops.h"


struct IMUDATA {
    double timestamp;
    Eigen::Matrix<double,3,1> wm;
    Eigen::Matrix<double,3,1> am;
};


class Propagator
{

public:

    // Default constuctor
    Propagator(double sigmaw, double sigmawb, double sigmaa, double sigmaab) {
        this->sigma_w = sigmaw;
        this->sigma_wb = sigmawb;
        this->sigma_a = sigmaa;
        this->sigma_ab = sigmaab;
    }

    // Our feed function for IMU measurements
    void feed_imu(double timestamp, Eigen::Matrix<double,3,1> wm, Eigen::Matrix<double,3,1> am);

    // Our propagation function, will propagate to the next timestep
    CpiV1* propagate(double time0, double time1, Eigen::Matrix<double,3,1> bg_lin, Eigen::Matrix<double,3,1> ba_lin);


private:


    /**
     * Nice helper function that will linearly interpolate between two imu messages
     * This should be used instead of just "cutting" imu messages that bound the camera times
     * Give better time offset if we use this function....
     */
    IMUDATA interpolate_data(IMUDATA imu_1, IMUDATA imu_2, double timestamp) {
        // time-distance lambda
        double lambda = (timestamp-imu_1.timestamp)/(imu_2.timestamp-imu_1.timestamp);
        //cout << "lambda - " << lambda << endl;
        // interpolate between the two times
        IMUDATA data;
        data.timestamp = timestamp;
        data.am = (1-lambda)*imu_1.am+lambda*imu_2.am;
        data.wm = (1-lambda)*imu_1.wm+lambda*imu_2.wm;
        return data;
    }

    // Our history of IMU messages (time, angular, linear)
    std::vector<IMUDATA> imu_data;

    // Our noises
    double sigma_w; // gyro white noise
    double sigma_wb; // gyro bias walk
    double sigma_a; // accel white noise
    double sigma_ab; // accel bias walk


};




#endif /* PROPAGATOR_H */

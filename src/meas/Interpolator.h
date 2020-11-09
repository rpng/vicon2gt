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


#ifndef INTERPOLATOR_H
#define INTERPOLATOR_H


#include <vector>
#include <algorithm>
#include <Eigen/Eigen>
#include <ros/ros.h>

#include "cpi/CpiV1.h"
#include "utils/quat_ops.h"


struct POSEDATA {
    double timestamp = -1;
    bool has_odom = false;
    // ori, pos, vel, ang
    Eigen::Matrix<double,4,1> q;
    Eigen::Matrix<double,3,1> p;
    Eigen::Matrix<double,3,1> v;
    Eigen::Matrix<double,3,1> w;
    // noise covariances
    Eigen::Matrix<double,3,3> R_q;
    Eigen::Matrix<double,3,3> R_p;
    Eigen::Matrix<double,3,3> R_v;
    Eigen::Matrix<double,3,3> R_w;
    // Comparison operator
    bool operator< ( const POSEDATA& s ) const { return timestamp < s.timestamp; }
    bool operator> ( const POSEDATA& s ) const { return timestamp > s.timestamp; }
};


class Interpolator
{

public:

    // Default constuctor
    Interpolator() { }

    // Our feed function for POSE measurements
    void feed_pose(double timestamp, Eigen::Matrix<double,4,1> q, Eigen::Matrix<double,3,1> p,
                   Eigen::Matrix<double,3,3> R_q, Eigen::Matrix<double,3,3> R_p);

    // Our feed function for ODOM measurements
    void feed_odom(double timestamp, Eigen::Matrix<double,4,1> q, Eigen::Matrix<double,3,1> p,
                   Eigen::Matrix<double,3,1> v, Eigen::Matrix<double,3,1> w,
                   Eigen::Matrix<double,3,3> R_q, Eigen::Matrix<double,3,3> R_p,
                   Eigen::Matrix<double,3,3> R_v, Eigen::Matrix<double,3,3> R_w);

    // Given a timestamp, this will get the pose at that time
    // If we don't have that pose in our vector, we will perform interpolation to get it
    bool get_pose(double timestamp, Eigen::Matrix<double,4,1>& q, Eigen::Matrix<double,3,1>& p, Eigen::Matrix<double,6,6>& R);

    // Given a timestamp, this will get the pose at that time
    // If we don't have that pose in our vector, we will perform interpolation to get it
    bool get_pose_with_jacobian(double timestamp, Eigen::Matrix<double,4,1>& q, Eigen::Matrix<double,3,1>& p, Eigen::Matrix<double,6,6>& R, Eigen::Matrix<double,6,1>& H_toff);

    // Given a timestamp, this will find the bounding poses for them
    bool get_bounds(double timestamp,
            double &time0, Eigen::Matrix<double,4,1>& q0, Eigen::Matrix<double,3,1>& p0, Eigen::Matrix<double,6,6>& R0,
            double &time1, Eigen::Matrix<double,4,1>& q1, Eigen::Matrix<double,3,1>& p1, Eigen::Matrix<double,6,6>& R1);


private:

    // Our history of POSE messages (time, ori, pos, vel, ang)
    // Note that this is sorted by timestamps so we can binary search through it....
    std::set<POSEDATA,std::less<POSEDATA>> pose_data;



};




#endif /* INTERPOLATOR_H */

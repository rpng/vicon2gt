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


#include "Interpolator.h"



/**
 * This should append incoming POSE messages to our stored data array
 */
void Interpolator::feed_pose(double timestamp, Eigen::Matrix<double,4,1> q, Eigen::Matrix<double,3,1> p,
                             Eigen::Matrix<double,3,3> R_q, Eigen::Matrix<double,3,3> R_p) {

    // Create our imu data object
    POSEDATA data;
    data.timestamp = timestamp;
    data.has_odom = false;
    data.q = q;
    data.p = p;
    data.R_q = R_q;
    data.R_p = R_p;

    // Append it to our vector
    pose_data.emplace_back(data);

}


/**
 * This should append incoming ODOM messages to our stored data array
 */
void Interpolator::feed_odom(double timestamp, Eigen::Matrix<double,4,1> q, Eigen::Matrix<double,3,1> p,
                             Eigen::Matrix<double,3,1> v, Eigen::Matrix<double,3,1> w,
                             Eigen::Matrix<double,3,3> R_q, Eigen::Matrix<double,3,3> R_p,
                             Eigen::Matrix<double,3,3> R_v, Eigen::Matrix<double,3,3> R_w) {

    // Create our imu data object
    POSEDATA data;
    data.timestamp = timestamp;
    data.has_odom = true;
    data.q = q;
    data.p = p;
    data.v = v;
    data.w = w;
    data.R_q = R_q;
    data.R_p = R_p;
    data.R_v = R_v;
    data.R_w = R_w;

    // Append it to our vector
    pose_data.emplace_back(data);

}




/**
 * Given a timestamp, this will get the pose at that time
 * If we do not have the pose, we will perform interpolation of two bounding measurements to find it
 * We also return the measurement covariance associated with this pose
 */
bool Interpolator::get_pose(double timestamp, Eigen::Matrix<double,4,1>& q,
                            Eigen::Matrix<double,3,1>& p, Eigen::Matrix<double,6,6>& R) {

    // Set the default values
    double time0 = -1;
    double time1 = -1;
    POSEDATA pose0, pose1, poseEXACT;

    // Find the bounding poses
    double min_time = -INFINITY;
    double max_time = INFINITY;
    bool found_exact = false;
    bool found_older = false;
    bool found_newer = false;

    //Find the bounding poses for interpolation. If no older one is found, measurement is unusable
    for (size_t i=0; i<pose_data.size(); i++) {
        if (pose_data.at(i).timestamp == timestamp){
            poseEXACT = pose_data.at(i);
            found_exact = true;
        }
        if (pose_data.at(i).timestamp > min_time && pose_data.at(i).timestamp <= timestamp){
            pose0 = pose_data.at(i);
            min_time = pose_data.at(i).timestamp;
            found_older = true;
        }
        if (pose_data.at(i).timestamp < max_time && pose_data.at(i).timestamp >= timestamp){
            pose1 = pose_data.at(i);
            max_time = pose_data.at(i).timestamp;
            found_newer = true;
        }
    }

    // If we found an exact one, just return that
    if(found_exact) {
        // mean values
        q = poseEXACT.q;
        p = poseEXACT.p;
        // meas covariance
        R.setZero();
        R.block(0,0,3,3) = poseEXACT.R_q;
        R.block(3,3,3,3) = poseEXACT.R_p;
        return true;
    }


    // Return false if we do not have any bounding pose for this measurement (shouldn't happen)
    if(!found_older || !found_newer || min_time == max_time) {
        ROS_ERROR("[INTERPOLATOR]: UNABLE TO FIND BOUNDING POSES");
        ROS_ERROR("[INTERPOLATOR]: tmeas = %.9f | time0 = %.9f | time1 = %.9f", timestamp, time0, time1);
        return false;
    }

    // Our lamda time-distance fraction
    double lambda = (timestamp-pose0.timestamp)/(pose1.timestamp-pose0.timestamp);

    // Bounding SO(3) orientations
    Eigen::Matrix<double,3,3> R_Gto0 = quat_2_Rot(pose0.q);
    Eigen::Matrix<double,3,3> R_Gto1 = quat_2_Rot(pose1.q);

    // Now perform the interpolation
    Eigen::Matrix<double,3,3> R_0to1 = R_Gto1*R_Gto0.transpose();
    Eigen::Matrix<double,3,3> R_0toi = Exp(lambda*vee(Log(R_0to1)));
    Eigen::Matrix<double,3,3> R_interp = R_0toi*R_Gto0;
    Eigen::Matrix<double,3,1> p_interp = (1-lambda)*pose0.p + lambda*pose1.p;

    // Calculate intermediate values for cov propagation equations
    // Equation (8)-(10) of Geneva2018ICRA async measurement paper
    Eigen::Matrix<double,3,3> eye33 = Eigen::Matrix<double,3,3>::Identity();
    Eigen::Matrix<double,3,3> JR_r0i = Jr(lambda*vee(Log(R_0to1)));
    Eigen::Matrix<double,3,3> JRinv_r01 = Jr(vee(Log(R_0to1))).inverse();
    JRinv_r01 = JRinv_r01.inverse();
    Eigen::Matrix<double,3,3> JRneg_r0i = Jr(-lambda*vee(Log(R_0to1.transpose())));
    Eigen::Matrix<double,3,3> JRneginv_r01 = Jr(vee(Log(R_0to1.transpose()))).inverse();
    JRneginv_r01 = JRneginv_r01.inverse();

    // Covariance propagation Jacobian
    // Equation (7) of Geneva2018ICRA async measurement paper
    Eigen::Matrix<double,6,12> Hu = Eigen::Matrix<double,6,12>::Zero();
    Hu.block(0,0,3,3) = -R_0toi*(JR_r0i*lambda*JRinv_r01-eye33);
    Hu.block(0,6,3,3) = R_0toi*(JRneg_r0i*lambda*JRinv_r01);
    Hu.block(3,6,3,3) = (1-lambda)*eye33;
    Hu.block(3,9,3,3) = lambda*eye33;

    // Finally propagate the covariance!
    Eigen::Matrix<double,12,12> R_12 = Eigen::Matrix<double,12,12>::Zero();
    R_12.block(0,0,3,3) = pose0.R_q;
    R_12.block(3,3,3,3) = pose0.R_p;
    R_12.block(6,6,3,3) = pose1.R_q;
    R_12.block(9,9,3,3) = pose1.R_p;
    R = Hu*R_12*Hu.transpose();

    // Done
    q = rot_2_quat(R_interp);
    p = p_interp;
    return true;
}

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


#include "MeasBased_ViconPoseFactor.h"


using namespace std;
using namespace gtsam;


/**
 * Called on when optimizing to get the error of this measurement
 */
gtsam::Vector MeasBased_ViconPoseFactor::evaluateError(const JPLNavState& state, const Rot3& R_BtoI, const Vector3& p_BinI,
                                                       boost::optional<Matrix&> H1, boost::optional<Matrix&> H2, boost::optional<Matrix&> H3) const {


    //================================================================================
    //================================================================================
    //================================================================================

    // Separate our variables from our states
    JPLQuaternion q_VtoI = state.q();
    Vector3 p_IinV = state.p();
    JPLQuaternion q_BtoI = rot_2_quat(R_BtoI.matrix());

    // Calculate the expected measurement values from the state
    JPLQuaternion q_VtoB = quat_multiply(Inv(q_BtoI),q_VtoI);
    Eigen::Matrix<double,3,1> p_BinV = p_IinV + quat_2_Rot(q_VtoI).transpose()*p_BinI;

    //================================================================================
    //================================================================================
    //================================================================================


    // Our error vector [delta = (orientation, position)]
    Vector6 error;

    // Error in our orientation
    JPLQuaternion q_r = quat_multiply(q_VtoB, Inv(mq_VtoB));

    // Error in our our state in respect to the measurement
    error.block(0,0,3,1) = 2*q_r.block(0,0,3,1);
    error.block(3,0,3,1) = p_BinV - mp_BinV;


    //================================================================================
    //================================================================================
    //================================================================================



    // Compute the Jacobian in respect to the first JPLNavState if needed
    if(H1) {
        Eigen::Matrix<double,6,15> H = Eigen::Matrix<double,6,15>::Zero();
        H.block(0,0,3,3) = quat_2_Rot(Inv(q_BtoI));
        H.block(3,0,3,3) = -quat_2_Rot(Inv(q_VtoI))*skew_x(p_BinI);
        H.block(3,12,3,3) = Eigen::Matrix<double,3,3>::Identity();
        *H1 = *OptionalJacobian<6,15>(H);
    }

    // Compute the Jacobian in respect orientation extrinics between BODY and IMU frames
    // NOTE:!@#!@#!@##@!@#!#!@#!@#!@#!@#!#!@#!@#!@#!@#!@#!@#!@#!$%$!@%$@!#$!@#!@#
    // NOTE: gtsam uses the right expodential expansion of the rotation error
    // NOTE: (I-skew(theta1)*R_VtoB = (R_BtoI*(I+skew(theta2)))^T*R_VtoI
    if(H2) {
        Eigen::Matrix<double,6,3> H = Eigen::Matrix<double,6,3>::Zero();
        //H.block(0,0,3,3) = -quat_2_Rot(Inv(q_BtoI)); // our form
        H.block(0,0,3,3).setIdentity(); // since we use gtsam rot3
        *H2 = *OptionalJacobian<6,3>(H);
    }

    // Compute the Jacobian in respect position extrinics between BODY and IMU frames
    if(H3) {
        Eigen::Matrix<double,6,3> H = Eigen::Matrix<double,6,3>::Zero();
        H.block(3,0,3,3) = quat_2_Rot(Inv(q_VtoI));
        *H3 = *OptionalJacobian<6,3>(H);
    }


    // Debug printing of error and Jacobians
    //if(H1) cout << endl << "ViconPoseFactor H1" << endl << *H1 << endl << endl;
    //if(H2) cout << endl << "ViconPoseFactor H2" << endl << *H2 << endl << endl;
    //if(H3) cout << endl << "ViconPoseFactor H3" << endl << *H3 << endl << endl;
    //KeyFormatter keyFormatter = DefaultKeyFormatter;
    //cout << endl << "ViconPoseFactor (" << keyFormatter(this->key1()) << "," << keyFormatter(this->key2()) << "," << keyFormatter(this->key3()) << ")" << endl << error.transpose() << endl;

    // Finally return our error vector!
    return error;
}







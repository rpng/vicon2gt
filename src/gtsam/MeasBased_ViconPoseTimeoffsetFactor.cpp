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


#include "MeasBased_ViconPoseTimeoffsetFactor.h"


using namespace std;
using namespace gtsam;


/**
 * Called on when optimizing to get the error of this measurement
 */
gtsam::Vector MeasBased_ViconPoseTimeoffsetFactor::evaluateError(const JPLNavState& state, const Rot3& R_BtoI, const Vector3& p_BinI, const Vector1& t_off,
                                                                 boost::optional<Matrix&> H1, boost::optional<Matrix&> H2, boost::optional<Matrix&> H3, boost::optional<Matrix&> H4) const {


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

    // Get our interpolated pose at the node timestep
    Eigen::Matrix<double,4,1> q_interp;
    Eigen::Matrix<double,3,1> p_interp;
    Eigen::Matrix<double,6,6> R_interp;
    Eigen::Matrix<double,6,1> H_toff;
    bool has_vicon = m_interpolator->get_pose_with_jacobian(m_time+t_off(0),q_interp,p_interp,R_interp, H_toff);

    // Find the sqrt inverse to whittening
    // This is because our measurement noise can change every iteration based on interpolation
    Eigen::Matrix<double,6,6> sqrt_inv_interp = R_interp.llt().matrixL();
    sqrt_inv_interp = sqrt_inv_interp.colPivHouseholderQr().solve(Eigen::Matrix<double,6,6>::Identity());

    // Our error vector [delta = (orientation, position)]
    Vector6 error;
    JPLQuaternion q_r = quat_multiply(q_VtoB, Inv(q_interp));

    // Error in our our state in respect to the measurement
    error.block(0,0,3,1) = 2*q_r.block(0,0,3,1);
    error.block(3,0,3,1) = p_BinV - p_interp;
    error = sqrt_inv_interp*error;

    //================================================================================
    //================================================================================
    //================================================================================

    // Compute the Jacobian in respect to the first JPLNavState if needed
    if(H1) {
        //*H1 = numericalDerivative41<Vector,JPLNavState,Rot3,Vector3,Vector1>(
        //        boost::bind(&ViconPoseTimeoffsetFactor::evaluateError, this,
        //                    _1, _2, _3, _4,
        //                    boost::none, boost::none, boost::none, boost::none),
        //        state, R_BtoI, p_BinI, t_off);
        Eigen::Matrix<double,6,15> H = Eigen::Matrix<double,6,15>::Zero();
        if(has_vicon) {
            H.block(0,0,3,3) = quat_2_Rot(Inv(q_BtoI));
            H.block(3,0,3,3) = -quat_2_Rot(Inv(q_VtoI))*skew_x(p_BinI);
            H.block(3,12,3,3) = Eigen::Matrix<double,3,3>::Identity();
            H = sqrt_inv_interp*H;
        }
        *H1 = *OptionalJacobian<6,15>(H);
    }

    // Compute the Jacobian in respect orientation extrinics between BODY and IMU frames
    // NOTE:!@#!@#!@##@!@#!#!@#!@#!@#!@#!#!@#!@#!@#!@#!@#!@#!@#!$%$!@%$@!#$!@#!@#
    // NOTE: gtsam uses the right exponential expansion of the rotation error
    // NOTE: (I-skew(theta1))*R_VtoB = (R_BtoI*(I+skew(theta2)))^T*R_VtoI
    if(H2) {
        Eigen::Matrix<double,6,3> H = Eigen::Matrix<double,6,3>::Zero();
        if(has_vicon) {
            //H.block(0,0,3,3) = -quat_2_Rot(Inv(q_BtoI)); // our form
            H.block(0, 0, 3, 3).setIdentity(); // since we use gtsam rot3
            H = sqrt_inv_interp*H;
        }
        *H2 = *OptionalJacobian<6,3>(H);
        //*H2 = numericalDerivative42<Vector,JPLNavState,Rot3,Vector3,Vector1>(
        //        boost::bind(&ViconPoseTimeoffsetFactor::evaluateError, this,
        //                    _1, _2, _3, _4,
        //                    boost::none, boost::none, boost::none, boost::none),
        //        state, R_BtoI, p_BinI, t_off);
    }

    // Compute the Jacobian in respect position extrinics between BODY and IMU frames
    if(H3) {
        Eigen::Matrix<double,6,3> H = Eigen::Matrix<double,6,3>::Zero();
        if(has_vicon) {
            H.block(3, 0, 3, 3) = quat_2_Rot(Inv(q_VtoI));
            H = sqrt_inv_interp*H;
        }
        *H3 = *OptionalJacobian<6,3>(H);
        //*H3 = numericalDerivative43<Vector,JPLNavState,Rot3,Vector3,Vector1>(
        //        boost::bind(&ViconPoseTimeoffsetFactor::evaluateError, this,
        //                    _1, _2, _3, _4,
        //                    boost::none, boost::none, boost::none, boost::none),
        //        state, R_BtoI, p_BinI, t_off);
    }


    // Compute the Jacobian in respect time offset
    // NOTE: the measurement itself is a function of time offset
    // NOTE: thus we here bring it over to the "other side" of the measurement equation and thus make it negative
    if(H4) {
        Eigen::Matrix<double,6,1> H = Eigen::Matrix<double,6,1>::Zero();
        if(has_vicon) {
            //H.block(0,0,3,1) = Jr(lambda*vee(Log(R_0to1))).transpose()*vee(Log(R_0to1))/(m_timeB1-m_timeB0);
            //H.block(3,0,3,1) = (mp_B0inV-mp_B1inV)/(m_timeB1-m_timeB0);
            //cout << endl << "ViconPoseTimeoffsetFactor H4 (numerical)" << endl << (H4->transpose()) << endl;
            //cout << endl << "ViconPoseTimeoffsetFactor H4 (analytical)" << endl << H_toff.transpose() << endl << endl;
            H = -H_toff;
            H = sqrt_inv_interp*H;
        }
        *H4 = *OptionalJacobian<6,1>(H);
        //*H4 = numericalDerivative44<Vector,JPLNavState,Rot3,Vector3,Vector1>(
        //        boost::bind(&ViconPoseTimeoffsetFactor::evaluateError, this,
        //                    _1, _2, _3, _4,
        //                    boost::none, boost::none, boost::none, boost::none),
        //        state, R_BtoI, p_BinI, t_off, 1e-5);
    }

    // Debug printing of error and Jacobians
    //if(H1) cout << endl << "ViconPoseTimeoffsetFactor H1" << endl << *H1 << endl << endl;
    //if(H2) cout << endl << "ViconPoseTimeoffsetFactor H2" << endl << *H2 << endl << endl;
    //if(H3) cout << endl << "ViconPoseTimeoffsetFactor H3" << endl << *H3 << endl << endl;
    //if(H4) cout << endl << "ViconPoseTimeoffsetFactor H4" << endl << *H4 << endl << endl;
    //KeyFormatter keyFormatter = DefaultKeyFormatter;
    //cout << endl << "ViconPoseFactor (" << keyFormatter(this->key1()) << "," << keyFormatter(this->key2()) << "," << keyFormatter(this->key3()) << ")" << endl << error.transpose() << endl;

    // Finally return our error vector!
    return error;
}







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


#ifndef GTSAM_VICONPOSETIMEOFFSETFACTOR_H
#define GTSAM_VICONPOSETIMEOFFSETFACTOR_H

#include <gtsam/base/debug.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/numericalDerivative.h>

#include "JPLNavState.h"
#include "utils/quat_ops.h"

using namespace gtsam;

namespace gtsam {


    /// JPL quaternion for the orientation
    typedef Eigen::Matrix<double,4,1> JPLQuaternion;

    /**
     * Vicon pose factor.
     * This is a measurement of the vicon "body" frame on the object in the vicon frame
     * Our states we are estimating are the JPL imu state, so we need the transform to it also
     */
    class ViconPoseTimeoffsetFactor : public NoiseModelFactor4<JPLNavState, Rot3, Vector3, Vector1> {
    private:

        double m_time; ///< time in the vicon clock that those pose should be at

        double m_timeB0; /// < time the first pose occured at
        JPLQuaternion mq_VtoB0; ///< orientation of this pose in the vicon frame
        Vector3 mp_B0inV; ///< position of the body in the vicon frame

        double m_timeB1; /// < time the second pose occured at
        JPLQuaternion mq_VtoB1; ///< orientation of this pose in the vicon frame
        Vector3 mp_B1inV; ///< position of the body in the vicon frame


    public:

        /// Construct from the two linking JPLNavStates, preingration measurement, and its covariance
        ViconPoseTimeoffsetFactor(Key kstate, Key kR_BtoI, Key kp_BinI, Key kt_off, Eigen::Matrix<double,6,6> covariance, double timestamp, double timeB0, JPLQuaternion q_VtoB0, Vector3 p_B0inV, double timeB1, JPLQuaternion q_VtoB1, Vector3 p_B1inV) :
                NoiseModelFactor4<JPLNavState, Rot3, Vector3, Vector1>(noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.345), noiseModel::Gaussian::Covariance(covariance)), kstate, kR_BtoI, kp_BinI, kt_off) {
            this->m_time = timestamp;
            this->m_timeB0 = timeB0;
            this->mq_VtoB0 = q_VtoB0;
            this->mp_B0inV = p_B0inV;
            this->m_timeB1 = timeB1;
            this->mq_VtoB1 = q_VtoB1;
            this->mp_B1inV = p_B1inV;
        }

        /// Rotation from vicon
        JPLQuaternion q_VtoB0() const {
            return mq_VtoB0;
        }

        /// Position in the vicon frame
        Vector3 p_B0inV() const {
            return mp_B0inV;
        }


        /// Rotation from vicon
        JPLQuaternion q_VtoB1() const {
            return mq_VtoB1;
        }

        /// Position in the vicon frame
        Vector3 p_B1inV() const {
            return mp_B1inV;
        }


        /// Error function. Given the current states, calculate the measurement error/residual
        gtsam::Vector evaluateError(const JPLNavState& state, const Rot3& R_BtoI, const Vector3& p_BinI, const Vector1& t_off,
                                    boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none,
                                    boost::optional<Matrix&> H3 = boost::none, boost::optional<Matrix&> H4 = boost::none) const;


        /// How this factor gets printed in the ostream
        GTSAM_EXPORT
        friend std::ostream &operator<<(std::ostream &os, const ViconPoseTimeoffsetFactor& factor) {
            os << "mq_VtoB0:[" << factor.q_VtoB0()(0) << ", " << factor.q_VtoB0()(1) << ", " << factor.q_VtoB0()(2) << ", " << factor.q_VtoB0()(3) << "]'" << endl;
            os << "mp_B0inV:[" << factor.p_B0inV()(0) << ", " << factor.p_B0inV()(1) << ", " << factor.p_B0inV()(2) << "]'" << endl;
            os << "mq_VtoB1:[" << factor.q_VtoB1()(0) << ", " << factor.q_VtoB1()(1) << ", " << factor.q_VtoB1()(2) << ", " << factor.q_VtoB1()(3) << "]'" << endl;
            os << "mp_B1inV:[" << factor.p_B1inV()(0) << ", " << factor.p_B1inV()(1) << ", " << factor.p_B1inV()(2) << "]'" << endl;
            return os;
        }

        /// Print function for this factor
        void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
            std::cout << s << "ViconPoseFactor(" << keyFormatter(this->key1()) << "," << keyFormatter(this->key2()) << "," << keyFormatter(this->key3()) << ")" << std::endl;
            std::cout << "  measured: " << std::endl << *this << std::endl;
            this->noiseModel_->print("  noise model: ");
        }

        /// Define how two factors can be equal to each other
        bool equals(const NonlinearFactor &expected, double tol = 1e-9) const {
            // Cast the object
            const ViconPoseTimeoffsetFactor *e =  dynamic_cast<const ViconPoseTimeoffsetFactor*>(&expected);
            if(e == NULL) return false;
            // Success, compare base noise values and the measurement values
            return NoiseModelFactor4<JPLNavState,Rot3,Vector3,Vector1>::equals(*e, tol)
                   && gtsam::equal(mq_VtoB0, e->mq_VtoB0, tol)
                   && gtsam::equal(mp_B0inV, e->mp_B0inV, tol)
                   && gtsam::equal(mq_VtoB1, e->mq_VtoB1, tol)
                   && gtsam::equal(mp_B1inV, e->mp_B1inV, tol);
        }

    };


} // namespace gtsam


#endif /* GTSAM_VICONPOSETIMEOFFSETFACTOR_H */
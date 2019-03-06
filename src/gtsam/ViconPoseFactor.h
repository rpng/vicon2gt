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


#ifndef GTSAM_VICONPOSEFACTOR_H
#define GTSAM_VICONPOSEFACTOR_H

#include <gtsam/base/debug.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

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
    class ViconPoseFactor : public NoiseModelFactor3<JPLNavState, Rot3, Vector3> {
    private:


        JPLQuaternion mq_VtoB; ///< orientation of this pose in the vicon frame
        Vector3 mp_BinV; ///< position of the body in the vicon frame


    public:

        /// Construct from the two linking JPLNavStates, preingration measurement, and its covariance
        ViconPoseFactor(Key kstate, Key kR_BtoI, Key kp_BinI, Eigen::Matrix<double,6,6> covariance, JPLQuaternion q_VtoB, Vector3 p_BinV) :
                NoiseModelFactor3<JPLNavState, Rot3, Vector3>(noiseModel::Gaussian::Covariance(covariance), kstate, kR_BtoI, kp_BinI) {
            this->mq_VtoB = q_VtoB;
            this->mp_BinV = p_BinV;
        }

        /// Rotation from vicon
        JPLQuaternion q_VtoB() const {
            return mq_VtoB;
        }

        /// Position in the vicon frame
        Vector3 p_BinV() const {
            return mp_BinV;
        }


        /// Error function. Given the current states, calculate the measurement error/residual
        gtsam::Vector evaluateError(const JPLNavState& state, const Rot3& R_BtoI, const Vector3& p_BinI,
                                    boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none,
                                    boost::optional<Matrix&> H3 = boost::none) const;


        /// How this factor gets printed in the ostream
        GTSAM_EXPORT
        friend std::ostream &operator<<(std::ostream &os, const ViconPoseFactor& factor) {
            os << "mq_VtoB:[" << factor.q_VtoB()(0) << ", " << factor.q_VtoB()(1) << ", " << factor.q_VtoB()(2) << ", " << factor.q_VtoB()(3) << "]'" << endl;
            os << "mp_BinV:[" << factor.p_BinV()(0) << ", " << factor.p_BinV()(1) << ", " << factor.p_BinV()(2) << "]'" << endl;
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
            const ViconPoseFactor *e =  dynamic_cast<const ViconPoseFactor*>(&expected);
            if(e == NULL) return false;
            // Success, compare base noise values and the measurement values
            return NoiseModelFactor3<JPLNavState,Rot3,Vector3>::equals(*e, tol)
                   && gtsam::equal(mq_VtoB, e->mq_VtoB, tol)
                   && gtsam::equal(mp_BinV, e->mp_BinV, tol);
        }

    };


} // namespace gtsam


#endif /* GTSAM_VICONPOSEFACTOR_H */
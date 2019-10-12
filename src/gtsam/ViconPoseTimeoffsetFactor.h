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
#include "meas/Interpolator.h"
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

        Interpolator* m_interpolator; ///< interpolator that has vicon poses in it


    public:

        /// Construct from the two linking JPLNavStates, preingration measurement, and its covariance
        ViconPoseTimeoffsetFactor(Key kstate, Key kR_BtoI, Key kp_BinI, Key kt_off, Eigen::Matrix<double,6,6> covariance, double timestamp, Interpolator* interpolator) :
                NoiseModelFactor4<JPLNavState, Rot3, Vector3, Vector1>(noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.345), noiseModel::Gaussian::Covariance(covariance)), kstate, kR_BtoI, kp_BinI, kt_off) {
            this->m_time = timestamp;
            this->m_interpolator = interpolator;
        }

        /// Timestamp we will interpolate to
        double time() const {
            return m_time;
        }


        /// Error function. Given the current states, calculate the measurement error/residual
        gtsam::Vector evaluateError(const JPLNavState& state, const Rot3& R_BtoI, const Vector3& p_BinI, const Vector1& t_off,
                                    boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none,
                                    boost::optional<Matrix&> H3 = boost::none, boost::optional<Matrix&> H4 = boost::none) const;


        /// How this factor gets printed in the ostream
        GTSAM_EXPORT
        friend std::ostream &operator<<(std::ostream &os, const ViconPoseTimeoffsetFactor& factor) {
            os << "m_time:[" << factor.time() << "]'" << endl;
            return os;
        }

        /// Print function for this factor
        void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
            std::cout << s << "ViconPoseTimeoffsetFactor(" << keyFormatter(this->key1()) << "," << keyFormatter(this->key2()) << "," << keyFormatter(this->key3()) << "," << keyFormatter(this->key4()) << ")" << std::endl;
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
                   && gtsam::equal(m_time, e->m_time, tol);
        }

    };


} // namespace gtsam


#endif /* GTSAM_VICONPOSETIMEOFFSETFACTOR_H */
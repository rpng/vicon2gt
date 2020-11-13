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


#ifndef GTSAM_VICONPOSETIMEOFFSETFACTOR_H
#define GTSAM_VICONPOSETIMEOFFSETFACTOR_H

#include <gtsam/base/debug.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/numericalDerivative.h>

#include "GtsamConfig.h"
#include "JPLNavState.h"
#include "JPLQuaternion.h"
#include "meas/Interpolator.h"
#include "utils/quat_ops.h"

using namespace gtsam;

namespace gtsam {

    /**
     * @brief Vicon pose factor with time offset.
     * This is a measurement of the vicon "body" frame on the object in the vicon frame
     * Our states we are estimating are the JPL imu state, so we need the transform to it also
     */
    class MeasBased_ViconPoseTimeoffsetFactor : public NoiseModelFactor4<JPLNavState, JPLQuaternion, Vector3, Vector1> {
    private:

        std::shared_ptr<Interpolator> m_interpolator; ///< interpolator that has vicon poses in it
        std::shared_ptr<GtsamConfig> m_config; ///< config file for if we should estimate calibration

    public:

        /// Construct from the two linking JPLNavStates, preingration measurement, and its covariance
        MeasBased_ViconPoseTimeoffsetFactor(Key kstate, Key kR_BtoI, Key kp_BinI, Key kt_off,
                                            std::shared_ptr<Interpolator> interpolator, std::shared_ptr<GtsamConfig> config) :
                NoiseModelFactor4<JPLNavState, JPLQuaternion, Vector3, Vector1>(noiseModel::Robust::Create(
                        noiseModel::mEstimator::Huber::Create(1.345),
                         noiseModel::Gaussian::Covariance(Eigen::Matrix<double,6,6>::Identity())
                        ), kstate, kR_BtoI, kp_BinI, kt_off) {
            this->m_interpolator = interpolator;
            this->m_config = config;
        }

        /// Error function. Given the current states, calculate the measurement error/residual
        gtsam::Vector evaluateError(const JPLNavState& state, const JPLQuaternion& R_BtoI, const Vector3& p_BinI, const Vector1& t_off,
                                    boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none,
                                    boost::optional<Matrix&> H3 = boost::none, boost::optional<Matrix&> H4 = boost::none) const;


        /// How this factor gets printed in the ostream
        GTSAM_EXPORT
        friend std::ostream &operator<<(std::ostream &os, const MeasBased_ViconPoseTimeoffsetFactor& factor) {
            return os;
        }

        /// Print function for this factor
        void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
            std::cout << s << "ViconPoseTimeoffsetFactor(" << keyFormatter(this->key1()) << "," << keyFormatter(this->key2()) << "," << keyFormatter(this->key3()) << "," << keyFormatter(this->key4()) << ")" << std::endl;
            this->noiseModel_->print("  noise model: ");
        }

        /// Define how two factors can be equal to each other
        bool equals(const NonlinearFactor &expected, double tol = 1e-9) const {
            // Cast the object
            const auto *e =  dynamic_cast<const MeasBased_ViconPoseTimeoffsetFactor*>(&expected);
            if(e == nullptr) return false;
            // Success, compare base noise values and the measurement values
            return NoiseModelFactor4<JPLNavState,JPLQuaternion,Vector3,Vector1>::equals(*e, tol);
        }

    };


} // namespace gtsam


#endif /* GTSAM_VICONPOSETIMEOFFSETFACTOR_H */
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


#ifndef GTSAM_MAGNITUDEPRIOR_H
#define GTSAM_MAGNITUDEPRIOR_H

#include <gtsam/base/debug.h>
#include <gtsam/nonlinear/NonlinearFactor.h>



using namespace gtsam;

namespace gtsam {


    /**
     * Forces a vector to have a specified magnitude.
     * Example: force gravity vector to be 9.81 in magnitude
     */
    class MagnitudePrior : public NoiseModelFactor1<Vector3> {
    private:


        double m_mag; //< magnitude that we want this vector to be


    public:

        /// Default constructor
        MagnitudePrior(Key vec, Vector1 sigma, double m_mag) : NoiseModelFactor1<Vector3>(noiseModel::Gaussian::Covariance(sigma), vec) {
            this->m_mag = m_mag;
        }

        /// Magnitude
        double mag() const {
            return m_mag;
        }

        /// Error function. Given the current states, calculate the measurement error/residual
        gtsam::Vector evaluateError(const Vector3& vec, boost::optional<Matrix&> H1 = boost::none) const;


        /// How this factor gets printed in the ostream
        GTSAM_EXPORT
        friend std::ostream &operator<<(std::ostream &os, const MagnitudePrior& factor) {
            os << "mag:[" << factor.mag() << "]'" << std::endl;
            return os;
        }

        /// Print function for this factor
        void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
            std::cout << s << "MagnitudePrior(" << keyFormatter(this->key()) << ")" << std::endl;
            std::cout << "  measured: " << std::endl << *this << std::endl;
            this->noiseModel_->print("  noise model: ");
        }

        /// Define how two factors can be equal to each other
        bool equals(const NonlinearFactor &expected, double tol = 1e-9) const {
            // Cast the object
            const MagnitudePrior *e =  dynamic_cast<const MagnitudePrior*>(&expected);
            if(e == NULL) return false;
            // Success, compare base noise values and the measurement values
            return NoiseModelFactor1<Vector3>::equals(*e, tol)
                   && std::abs(e->m_mag-m_mag) < tol;
        }

    };


} // namespace gtsam


#endif /* GTSAM_MAGNITUDEPRIOR_H */
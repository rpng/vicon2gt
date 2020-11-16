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


#ifndef GTSAM_ROTATIONXY_H
#define GTSAM_ROTATIONXY_H


#include <Eigen/Eigen>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Manifold.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "utils/rpy_ops.h"


namespace gtsam {

    /**
     * @brief A 3d rotation with yaw fixed at zero
     * Estimate a 2 dof rotation => Rotx * Roty * Rotz(0)
     */
    class RotationXY {
    private:

        double theta_x; ///< Rotation about the x-axis
        double theta_y; ///< Rotation about the y-axis

    public:

        enum {
            dimension = 2
        };

        /// Default constructor
        RotationXY() : theta_x(0), theta_y(0) { }

        /// Construct from RotationXY directly
        RotationXY(const RotationXY& obj) {
            this->theta_x = obj.theta_x;
            this->theta_y = obj.theta_y;
        }

        /// Construct from two angles
        RotationXY(const double& thetax, const double& thetay) : theta_x(thetax), theta_y(thetay) { }

        /// Return x rotation theta.
        double thetax() const {
            return theta_x;
        }

        /// Return y rotation theta.
        double thetay() const {
            return theta_y;
        }

        /// Return full 3D rotation
        Eigen::Matrix3d rot() const {
            return rot_y(theta_y)*rot_x(theta_x);
        }

        /// Retract with optional derivatives (given correction)
        RotationXY retract(const Vector2& xi) const;

        /// Converting function from our over parameterization to the local representation (expanding about the current node's tangent space)
        Vector2 localCoordinates(const RotationXY& state) const;

        /// How this node gets printed in the ostream
        GTSAM_EXPORT
        friend std::ostream &operator<<(std::ostream &os, const RotationXY& state) {
            os << "thetax:[" << state.thetax() << "]'" << std::endl;
            os << "thetay:[" << state.thetay() << "]'" << std::endl;
            return os;
        }

        /// Print function for this node
        void print(const std::string& s = "") const {
            std::cout << s << *this << std::endl;
        }

        /// Equals function to compare this and another JPLNavState
        bool equals(const RotationXY& other, double tol = 1e-8) const {
            return gtsam::equal(theta_x, other.theta_x, tol)
                && gtsam::equal(theta_y, other.theta_y, tol);
        }


    };

    template<>
    struct traits<RotationXY> : internal::Manifold<RotationXY> { };


} // namespace gtsam


#endif /* GTSAM_ROTATIONXY_H */
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


#ifndef GTSAM_JPLQUATERNION_H
#define GTSAM_JPLQUATERNION_H


#include <Eigen/Eigen>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Manifold.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "utils/quat_ops.h"

namespace gtsam {

    /**
     * \brief JPL Navigation State
     * Contains orientation, position, velocity, and biases
     */
    class JPLQuaternion {
    private:

        Eigen::Matrix<double,4,1> q_GtoI; ///< Rotation from global to IMU

    public:

        enum {
            dimension = 3
        };

        /// Default constructor
        JPLQuaternion() : q_GtoI(0,0,0,1) { }

        /// Construct from JPLNavState directly
        JPLQuaternion(const JPLQuaternion& obj) {
            this->q_GtoI = obj.q_GtoI;
        }

        /// Construct from orientation, position, velocity, and biases
        JPLQuaternion(const Eigen::Matrix<double,4,1>& q) : q_GtoI(q) { }

        /// Return rotation quaternion.
        Eigen::Matrix<double,4,1> q() const {
            return q_GtoI;
        }

        /// Retract with optional derivatives (given correction)
        JPLQuaternion retract(const Vector3& xi) const;

        /// Converting function from our over parameterization to the local representation (expanding about the current node's tangent space)
        Vector3 localCoordinates(const JPLQuaternion& state) const;

        /// How this node gets printed in the ostream
        GTSAM_EXPORT
        friend std::ostream &operator<<(std::ostream &os, const JPLQuaternion& state) {
            os << "q:[" << state.q()(0) << ", " << state.q()(1) << ", " << state.q()(2) << ", " << state.q()(3) << "]'" << endl;
            return os;
        }

        /// Print function for this node
        void print(const std::string& s = "") const {
            cout << s << *this << endl;
        }

        /// Equals function to compare this and another JPLNavState
        bool equals(const JPLQuaternion& other, double tol = 1e-8) const {
            return gtsam::equal(q_GtoI, other.q_GtoI, tol);
        }


    };

    template<>
    struct traits<JPLQuaternion> : internal::Manifold<JPLQuaternion> { };


} // namespace gtsam


#endif /* GTSAM_JPLQUATERNION_H */
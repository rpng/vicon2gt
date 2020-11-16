/**
 * MIT License
 * Copyright (c) 2018 Patrick Geneva @ University of Delaware (Robot Perception & Navigation Group)
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
#ifndef RPYOPS_H
#define RPYOPS_H


#include <Eigen/Eigen>



/**
 * @brief Construct rotation matrix from given roll
 * @param t roll angle
 */
static inline Eigen::Matrix<double, 3, 3> rot_x(double t) {
    Eigen::Matrix<double, 3, 3> r;
    double ct = cos(t);
    double st = sin(t);
    r << 1.0, 0.0, 0.0, 0.0, ct, -st, 0.0, st, ct;
    return r;
}

/**
 * @brief Construct rotation matrix from given pitch
 * @param t pitch angle
 */
static inline Eigen::Matrix<double, 3, 3> rot_y(double t) {
    Eigen::Matrix<double, 3, 3> r;
    double ct = cos(t);
    double st = sin(t);
    r << ct, 0.0, st, 0.0, 1.0, 0.0, -st, 0.0, ct;
    return r;
}

/**
 * @brief Construct rotation matrix from given yaw
 * @param t yaw angle
 */
static inline Eigen::Matrix<double, 3, 3> rot_z(double t) {
    Eigen::Matrix<double, 3, 3> r;
    double ct = cos(t);
    double st = sin(t);
    r << ct, -st, 0.0, st, ct, 0.0, 0.0, 0.0, 1.0;
    return r;
}

/**
 * @brief Gets roll, pitch, yaw of argument rotation (in that order).
 * To recover the matrix: R_input = R_z(yaw)*R_y(pitch)*R_x(roll)
 * Rotation Definition: http://planning.cs.uiuc.edu/node102.html
 * Derivation: http://planning.cs.uiuc.edu/node103.html
 * @param rot Rotation matrix
 * @return [roll,pitch,yaw] values
 */
static inline Eigen::Matrix<double, 3, 1> rot2rpy(const Eigen::Matrix<double, 3, 3> &rot) {
    Eigen::Matrix<double, 3, 1> rpy;
    rpy(2) = atan2(rot(1,0),rot(0,0));
    rpy(1) = atan2(-rot(2,0),sqrt(rot(2,1)*rot(2,1)+rot(2,2)*rot(2,2)));
    rpy(0) = atan2(rot(2,1),rot(2,2));
    return rpy;
}

/**
 * @brief This will ensure that the angle theta is valid and in range [-pi,pi].
 * @param theta Input 1d rotation
 * @return Equivilent rotation within the range [-pi,pi]
 */
static inline double wrap2pi(double theta) {
    while(theta > M_PI) theta -= 2*M_PI;
    while(theta < -M_PI) theta += 2*M_PI;
    return theta;
}


#endif //RPYOPS_H


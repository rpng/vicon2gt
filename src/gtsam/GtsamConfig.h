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

#ifndef GTSAMCONFIG_H
#define GTSAMCONFIG_H

#include <Eigen/Eigen>


/**
 * @brief Configuration object that informs the factors if they need to compute Jacobians.
 *
 * For some reason GTSAM doesn't seem to support "fixing" of variables, thus we would need to make a bunch of factors.
 * We instead will always add calibration factors to the graph (which might make it slower), but then just zero the Jacobians.
 * Zero'ing the Jacobians in respect to the calibration will allow them to remain the same (i.e. the factors are not a function of).
 */
struct GtsamConfig {

public:

    /// If we want to estimate the time offset between VICON and IMU
    bool estimate_vicon_imu_toff = true;

    /// If we want to estimate the orientation between VICON and IMU
    bool estimate_vicon_imu_ori = true;

    /// If we want to estimate the position between VICON and IMU
    bool estimate_vicon_imu_pos = true;

};

#endif //GTSAMCONFIG_H
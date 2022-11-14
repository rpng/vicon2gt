/*
 * The vicon2gt project
 * Copyright (C) 2020 Patrick Geneva
 * Copyright (C) 2020 Guoquan Huang
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "JPLQuaternion.h"

using namespace std;
using namespace gtsam;

JPLQuaternion gtsam::JPLQuaternion::retract(const Vector3 &xi) const {

  // Calculate the update quaternion from the minimal representation
  Eigen::Vector3d dth = xi.block(0, 0, 3, 1);
  Eigen::Vector3d dq13 = ((std::sin(dth.norm() / 2) / dth.norm())) * dth;
  double dq4 = std::cos(dth.norm() / 2);

  // From the minimal representation, create the full 4x1 correction quaternion
  Eigen::Vector4d dq;
  dq << dq13, dq4;
  dq = dq / dq.norm();
  if (dq(3) < 0) {
    dq = -dq;
  }

  // Ensure that quaternion is valid
  if (std::isnan(dq.norm())) {
    dq << 0, 0, 0, 1.0;
  }

  // Update our current state values
  Eigen::Vector4d q = quat_multiply(dq, q_GtoI);

  // Reconstruct and return this new state
  return JPLQuaternion(q);
}

Vector3 gtsam::JPLQuaternion::localCoordinates(const JPLQuaternion &state) const {
  return 2 * quat_multiply(state.q(), Inv(q_GtoI)).block(0, 0, 3, 1);
}

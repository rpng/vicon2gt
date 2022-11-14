/*
 * The vicon2gt project
 * Copyright (C) 2020 Patrick Geneva
 * Copyright (C) 2020 Kevin Eckenhoff
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
#include "JPLNavState.h"

using namespace std;
using namespace gtsam;

JPLNavState gtsam::JPLNavState::retract(const Vector15 &xi) const {

  // Calculate the update quaternion from the minimal representation
  Eigen::Vector3d dth = xi.block(0, 0, 3, 1);
  Eigen::Vector3d dq13 = ((std::sin(dth.norm() / 2) / dth.norm())) * dth;
  double dq4 = std::cos(dth.norm() / 2);

  // From the minimal representation, create the full 4x1 correction quaternion
  Vector4 dq;
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
  Vector4 q = quat_multiply(dq, q_GtoI);
  Bias3 bg = biasg + xi.block(3, 0, 3, 1);
  Velocity3 v = v_IinG + xi.block(6, 0, 3, 1);
  Bias3 ba = biasa + xi.block(9, 0, 3, 1);
  Vector3 p = p_IinG + xi.block(12, 0, 3, 1);

  // Reconstruct and return this new state
  return JPLNavState(m_time, q, bg, v, ba, p);
}

Vector15 gtsam::JPLNavState::localCoordinates(const JPLNavState &state) const {
  Vector15 localrep;
  localrep.block(0, 0, 3, 1) = 2 * quat_multiply(state.q(), Inv(q_GtoI)).block(0, 0, 3, 1);
  localrep.block(3, 0, 3, 1) = state.bg() - biasg;
  localrep.block(6, 0, 3, 1) = state.v() - v_IinG;
  localrep.block(9, 0, 3, 1) = state.ba() - biasa;
  localrep.block(12, 0, 3, 1) = state.p() - p_IinG;
  return localrep;
}

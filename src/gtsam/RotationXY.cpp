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
#include "RotationXY.h"

using namespace std;
using namespace gtsam;

RotationXY gtsam::RotationXY::retract(const Vector2 &xi) const {

  // Calculate the update theta x y values
  double theta_new_x = wrap2pi(theta_x + wrap2pi(xi(0)));
  double theta_new_y = wrap2pi(theta_y + wrap2pi(xi(1)));

  // Reconstruct and return this new state
  return RotationXY(theta_new_x, theta_new_y);
}

Vector2 gtsam::RotationXY::localCoordinates(const RotationXY &state) const {
  Vector2 vec;
  vec(0) = theta_x;
  vec(1) = theta_y;
  return vec;
}

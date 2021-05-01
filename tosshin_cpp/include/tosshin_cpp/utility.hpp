// Copyright (c) 2021 Alfi Maulana
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef TOSSHIN_CPP__UTILITY_HPP_
#define TOSSHIN_CPP__UTILITY_HPP_

#include <keisan/keisan.hpp>
#include <tosshin_interfaces/msg/maneuver.hpp>
#include <tosshin_interfaces/msg/odometry.hpp>
#include <tosshin_interfaces/msg/orientation.hpp>
#include <tosshin_interfaces/msg/position.hpp>
#include <tosshin_interfaces/srv/configure_odometry.hpp>
#include <tosshin_interfaces/srv/configure_maneuver.hpp>

namespace tosshin_cpp
{

using Maneuver = tosshin_interfaces::msg::Maneuver;
using Odometry = tosshin_interfaces::msg::Odometry;
using Orientation = tosshin_interfaces::msg::Orientation;
using Position = tosshin_interfaces::msg::Position;
using ConfigureOdometry = tosshin_interfaces::srv::ConfigureOdometry;
using ConfigureManeuver = tosshin_interfaces::srv::ConfigureManeuver;

inline keisan::Point2 position_to_point(const Position & position)
{
  return keisan::Point2(position.x, position.y);
}

inline Position point_to_position(const keisan::Point2 & point)
{
  Position position;

  position.x = point.x;
  position.y = point.y;

  return position;
}

}  // namespace tosshin_cpp

#endif  // TOSSHIN_CPP__UTILITY_HPP_

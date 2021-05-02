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

#ifndef TOSSHIN_CPP__INTERFACE_HPP_
#define TOSSHIN_CPP__INTERFACE_HPP_

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

const char * const NAVIGATION_PREFIX = "/navigation";

const char * const ODOMETRY_SUFFIX = "/odometry";

const char * const MANEUVER_INPUT_SUFFIX = "/maneuver_input";
const char * const MANEUVER_EVENT_SUFFIX = "/maneuver_event";

const char * const CONFIGURE_MANEUVER_SUFFIX = "/configure_maneuver";
}  // namespace tosshin_cpp

#endif  // TOSSHIN_CPP__INTERFACE_HPP_

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

#ifndef TOSSHIN_CPP__NAVIGATION_PROVIDER_HPP_
#define TOSSHIN_CPP__NAVIGATION_PROVIDER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tosshin_interfaces/tosshin_interfaces.hpp>

#include <memory>
#include <string>

namespace tosshin_cpp
{

using Maneuver = tosshin_interfaces::msg::Maneuver;
using Odometry = tosshin_interfaces::msg::Odometry;
using ConfigureManeuver = tosshin_interfaces::srv::ConfigureManeuver;

class NavigationProvider
{
public:
  NavigationProvider(rclcpp::Node::SharedPtr node);

protected:
  virtual const Maneuver & configure_maneuver(const Maneuver & maneuver);

  void set_odometry(const Odometry & odometry);
  void set_maneuver(const Maneuver & maneuver);

  const Maneuver & get_maneuver();

  rclcpp::Node::SharedPtr node;

private:
  rclcpp::Publisher<Odometry>::SharedPtr odometry_publisher;

  rclcpp::Publisher<Maneuver>::SharedPtr maneuver_event_publisher;
  rclcpp::Subscription<Maneuver>::SharedPtr maneuver_input_subscription;

  rclcpp::Service<ConfigureManeuver>::SharedPtr configure_maneuver_service;

  Maneuver current_maneuver;
};

}  // namespace tosshin_cpp

#endif  // TOSSHIN_CPP__NAVIGATION_PROVIDER_HPP_

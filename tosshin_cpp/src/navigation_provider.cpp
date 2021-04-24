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

#include <tosshin_cpp/navigation_provider.hpp>

#include <sys/socket.h>
#include <unistd.h>

#include <memory>
#include <string>
#include <vector>

namespace tosshin_cpp
{

NavigationProvider::NavigationProvider(rclcpp::Node::SharedPtr node)
: node(node),
  current_maneuver()
{
  // Initialize the odometry publisher
  {
    odometry_publisher = node->create_publisher<Odometry>("navigation/odometry", 10);

    RCLCPP_INFO_STREAM(
      node->get_logger(),
      "Odometry publisher initialized on " <<
        odometry_publisher->get_topic_name() << "!");
  }

  // Initialize the maneuver event publisher
  {
    maneuver_event_publisher = node->create_publisher<Maneuver>(
      "/navigation/maneuver_event", 10);

    RCLCPP_INFO_STREAM(
      node->get_logger(),
      "Maneuver event publisher initialized on " <<
        maneuver_event_publisher->get_topic_name() << "!");
  }

  // Initialize the maneuver input subscription
  {
    maneuver_input_subscription = node->create_subscription<Maneuver>(
      "/navigation/maneuver_input", 10,
      [this](const Maneuver::SharedPtr maneuver) {
        set_maneuver(*maneuver);
      });

    RCLCPP_INFO_STREAM(
      node->get_logger(),
      "Maneuver input subscription initialized on " <<
        maneuver_input_subscription->get_topic_name() << "!");
  }

  // Initialize the configure maneuver service
  {
    configure_maneuver_service = node->create_service<ConfigureManeuver>(
      "/navigation/configure_maneuver",
      [this](ConfigureManeuver::Request::SharedPtr request,
      ConfigureManeuver::Response::SharedPtr response) {
        bool configured = false;

        if (request->maneuver.size() > 0) {
          set_maneuver(request->maneuver.front());
          response->maneuver.push_back(get_maneuver());
          configured = true;
        }

        if (!configured) {
          response->maneuver.push_back(get_maneuver());
        }
      });

    RCLCPP_INFO_STREAM(
      node->get_logger(),
      "Configure maneuver service initialized on " <<
        configure_maneuver_service->get_service_name() << "!");
  }
}

const Maneuver & NavigationProvider::configure_maneuver(const Maneuver & maneuver)
{
  return maneuver;
}

void NavigationProvider::set_odometry(const Odometry & odometry)
{
  odometry_publisher->publish(odometry);
}

void NavigationProvider::set_maneuver(const Maneuver & maneuver)
{
  current_maneuver = configure_maneuver(maneuver);
  maneuver_event_publisher->publish(current_maneuver);
}

const Maneuver & NavigationProvider::get_maneuver()
{
  return current_maneuver;
}

}  // namespace tosshin_cpp

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

namespace tosshin_cpp
{

using Maneuver = tosshin_interfaces::msg::Maneuver;
using Odometry = tosshin_interfaces::msg::Odometry;
using ConfigureManeuver = tosshin_interfaces::srv::ConfigureManeuver;

class NavigationProvider
{
public:
  NavigationProvider();
  explicit NavigationProvider(rclcpp::Node::SharedPtr node);

protected:
  virtual const Maneuver & configure_maneuver(const Maneuver & maneuver);

  void set_node(rclcpp::Node::SharedPtr node);

  void set_odometry(const Odometry & odometry);
  void set_maneuver(const Maneuver & maneuver);

  rclcpp::Node::SharedPtr get_node();

  const Maneuver & get_maneuver();

private:
  rclcpp::Node::SharedPtr node;

  rclcpp::Publisher<Odometry>::SharedPtr odometry_publisher;

  rclcpp::Publisher<Maneuver>::SharedPtr maneuver_event_publisher;
  rclcpp::Subscription<Maneuver>::SharedPtr maneuver_input_subscription;

  rclcpp::Service<ConfigureManeuver>::SharedPtr configure_maneuver_service;

  Maneuver current_maneuver;
};

NavigationProvider::NavigationProvider()
{
}

NavigationProvider::NavigationProvider(rclcpp::Node::SharedPtr node)
{
  set_node(node);
}

const Maneuver & NavigationProvider::configure_maneuver(const Maneuver & maneuver)
{
  return maneuver;
}

void NavigationProvider::set_node(rclcpp::Node::SharedPtr node)
{
  // Initialize the node
  {
    this->node = node;

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Node initialized on " << get_node()->get_name() << "!");
  }

  // Initialize the odometry publisher
  {
    odometry_publisher = get_node()->create_publisher<Odometry>("navigation/odometry", 10);

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Odometry publisher initialized on " <<
        odometry_publisher->get_topic_name() << "!");
  }

  // Initialize the maneuver event publisher
  {
    maneuver_event_publisher = get_node()->create_publisher<Maneuver>(
      "/navigation/maneuver_event", 10);

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Maneuver event publisher initialized on " <<
        maneuver_event_publisher->get_topic_name() << "!");
  }

  // Initialize the maneuver input subscription
  {
    maneuver_input_subscription = get_node()->create_subscription<Maneuver>(
      "/navigation/maneuver_input", 10,
      [this](const Maneuver::SharedPtr maneuver) {
        set_maneuver(*maneuver);
      });

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Maneuver input subscription initialized on " <<
        maneuver_input_subscription->get_topic_name() << "!");
  }

  // Initialize the configure maneuver service
  {
    configure_maneuver_service = get_node()->create_service<ConfigureManeuver>(
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
      get_node()->get_logger(),
      "Configure maneuver service initialized on " <<
        configure_maneuver_service->get_service_name() << "!");
  }
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

rclcpp::Node::SharedPtr NavigationProvider::get_node()
{
  return node;
}

const Maneuver & NavigationProvider::get_maneuver()
{
  return current_maneuver;
}

}  // namespace tosshin_cpp

#endif  // TOSSHIN_CPP__NAVIGATION_PROVIDER_HPP_

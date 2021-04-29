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

#ifndef TOSSHIN_CPP__CONSUMER__NAVIGATION_CONSUMER_HPP_
#define TOSSHIN_CPP__CONSUMER__NAVIGATION_CONSUMER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tosshin_interfaces/tosshin_interfaces.hpp>

#include <memory>

namespace tosshin_cpp
{

using Maneuver = tosshin_interfaces::msg::Maneuver;
using Odometry = tosshin_interfaces::msg::Odometry;
using ConfigureManeuver = tosshin_interfaces::srv::ConfigureManeuver;

class NavigationConsumer
{
public:
  inline NavigationConsumer();
  inline explicit NavigationConsumer(rclcpp::Node::SharedPtr node);

  inline void set_node(rclcpp::Node::SharedPtr node);

  inline const Maneuver & configure_maneuver(const Maneuver & maneuver);
  inline const Maneuver & configure_maneuver_to_stop();

  inline void set_maneuver(const Maneuver & maneuver);

  inline rclcpp::Node::SharedPtr get_node();

  inline const Odometry & get_odometry();
  inline const Maneuver & get_maneuver();

private:
  rclcpp::Node::SharedPtr node;

  rclcpp::Subscription<Odometry>::SharedPtr odometry_subscription;

  rclcpp::Subscription<Maneuver>::SharedPtr maneuver_event_subscription;
  rclcpp::Publisher<Maneuver>::SharedPtr maneuver_input_publisher;

  rclcpp::Client<ConfigureManeuver>::SharedPtr configure_maneuver_client;

  Odometry current_odometry;
  Maneuver current_maneuver;
};

NavigationConsumer::NavigationConsumer()
{
}

NavigationConsumer::NavigationConsumer(rclcpp::Node::SharedPtr node)
{
  set_node(node);
}

void NavigationConsumer::set_node(rclcpp::Node::SharedPtr node)
{
  // Initialize the node
  {
    this->node = node;

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Node initialized on " << get_node()->get_name() << "!");
  }

  // Initialize the odometry subscription
  {
    odometry_subscription = get_node()->create_subscription<Odometry>(
      "navigation/odometry", 10,
      [this](const Odometry::SharedPtr msg) {
        current_odometry = *msg;
      });

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Odometry subscription initialized on " <<
        odometry_subscription->get_topic_name() << "!");
  }

  // Initialize the maneuver event subscription
  {
    maneuver_event_subscription = get_node()->create_subscription<Maneuver>(
      "/navigation/maneuver_event", 10,
      [this](const Maneuver::SharedPtr msg) {
        current_maneuver = *msg;
      });

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Maneuver event subscription initialized on " <<
        maneuver_event_subscription->get_topic_name() << "!");
  }

  // Initialize the maneuver input publisher
  {
    maneuver_input_publisher = get_node()->create_publisher<Maneuver>(
      "/navigation/maneuver_input", 10);

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Maneuver input publisher initialized on " <<
        maneuver_input_publisher->get_topic_name() << "!");
  }

  // Initialize the configure maneuver client
  {
    configure_maneuver_client = get_node()->create_client<ConfigureManeuver>(
      "/navigation/configure_maneuver");

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Configure maneuver client initialized on " <<
        configure_maneuver_client->get_service_name() << "!");

    // Request maneuver data
    {
      if (configure_maneuver_client->wait_for_service(std::chrono::seconds(1))) {
        auto request = std::make_shared<ConfigureManeuver::Request>();

        configure_maneuver_client->async_send_request(
          request, [this](rclcpp::Client<ConfigureManeuver>::SharedFuture future) {
            auto response = future.get();
            if (response->maneuver.size() > 0) {
              current_maneuver = response->maneuver.front();
            }
          });
      } else {
      }
    }
  }
}

const Maneuver & NavigationConsumer::configure_maneuver(const Maneuver & maneuver)
{
  if (!configure_maneuver_client->wait_for_service(std::chrono::seconds(1))) {
    throw std::runtime_error("configure maneuver service is not ready");
  }

  auto request = std::make_shared<ConfigureManeuver::Request>();
  request->maneuver.push_back(maneuver);

  auto future = configure_maneuver_client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(get_node(), future) != rclcpp::FutureReturnCode::SUCCESS) {
    throw std::runtime_error("failed to call configure maneuver service");
  }

  auto response = future.get();
  if (response->maneuver.size() < 1) {
    throw std::runtime_error("maneuver not configured");
  }

  current_maneuver = response->maneuver.front();

  return current_maneuver;
}

const Maneuver & NavigationConsumer::configure_maneuver_to_stop()
{
  Maneuver maneuver;

  maneuver.forward = 0.0;
  maneuver.left = 0.0;
  maneuver.yaw = 0.0;

  return configure_maneuver(maneuver);
}

void NavigationConsumer::set_maneuver(const Maneuver & maneuver)
{
  maneuver_input_publisher->publish(maneuver);
}

rclcpp::Node::SharedPtr NavigationConsumer::get_node()
{
  return node;
}

const Odometry & NavigationConsumer::get_odometry()
{
  return current_odometry;
}

const Maneuver & NavigationConsumer::get_maneuver()
{
  return current_maneuver;
}

}  // namespace tosshin_cpp

#endif  // TOSSHIN_CPP__CONSUMER__NAVIGATION_CONSUMER_HPP_

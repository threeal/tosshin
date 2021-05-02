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

#ifndef TOSSHIN_CPP__CONSUMER__MANEUVER_CONSUMER_HPP_
#define TOSSHIN_CPP__CONSUMER__MANEUVER_CONSUMER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <future>
#include <memory>
#include <string>

#include "../interface.hpp"

namespace tosshin_cpp
{

class ManeuverConsumer
{
public:
  using ManeuverCallback = std::function<void (const Maneuver &)>;

  inline ManeuverConsumer();

  inline explicit ManeuverConsumer(
    rclcpp::Node::SharedPtr node, const std::string & prefix = NAVIGATION_PREFIX);

  inline void set_node(
    rclcpp::Node::SharedPtr node, const std::string & prefix = NAVIGATION_PREFIX);

  inline void set_on_change_maneuver(const ManeuverCallback & callback);

  inline void request_to_configure_maneuver(
    ConfigureManeuver::Request::SharedPtr request, const ManeuverCallback & callback);

  inline void configure_maneuver(const Maneuver & maneuver, const ManeuverCallback & callback);
  inline std::shared_future<Maneuver> configure_maneuver(const Maneuver & maneuver);

  inline void fetch_maneuver(const ManeuverCallback & callback);
  inline std::shared_future<Maneuver> fetch_maneuver();

  inline void set_maneuver(const Maneuver & maneuver);

  inline rclcpp::Node::SharedPtr get_node() const;

  inline const Maneuver & get_maneuver() const;

private:
  inline void change_maneuver(const Maneuver & maneuver);

  rclcpp::Node::SharedPtr node;

  rclcpp::Subscription<Maneuver>::SharedPtr maneuver_event_subscription;
  rclcpp::Publisher<Maneuver>::SharedPtr maneuver_input_publisher;

  rclcpp::Client<ConfigureManeuver>::SharedPtr configure_maneuver_client;

  ManeuverCallback on_change_maneuver;

  Maneuver current_maneuver;
};

ManeuverConsumer::ManeuverConsumer()
{
}

ManeuverConsumer::ManeuverConsumer(rclcpp::Node::SharedPtr node, const std::string & prefix)
{
  set_node(node, prefix);
}

void ManeuverConsumer::set_node(rclcpp::Node::SharedPtr node, const std::string & prefix)
{
  // Initialize the node
  this->node = node;

  // Initialize the maneuver event subscription
  {
    maneuver_event_subscription = get_node()->create_subscription<Maneuver>(
      prefix + MANEUVER_EVENT_SUFFIX, 10,
      [this](const Maneuver::SharedPtr msg) {
        change_maneuver(*msg);
      });

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Maneuver event subscription initialized on `" <<
        maneuver_event_subscription->get_topic_name() << "`!");
  }

  // Initialize the maneuver input publisher
  {
    maneuver_input_publisher = get_node()->create_publisher<Maneuver>(
      prefix + MANEUVER_INPUT_SUFFIX, 10);

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Maneuver input publisher initialized on `" <<
        maneuver_input_publisher->get_topic_name() << "`!");
  }

  // Initialize the configure maneuver client
  {
    configure_maneuver_client = get_node()->create_client<ConfigureManeuver>(
      prefix + CONFIGURE_MANEUVER_SUFFIX);

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Configure maneuver client initialized on `" <<
        configure_maneuver_client->get_service_name() << "`!");
  }

  // Initial data fetch
  fetch_maneuver();
}

void ManeuverConsumer::set_on_change_maneuver(const ManeuverCallback & callback)
{
  on_change_maneuver = callback;
}

void ManeuverConsumer::request_to_configure_maneuver(
  ConfigureManeuver::Request::SharedPtr request, const ManeuverCallback & callback)
{
  if (!configure_maneuver_client->wait_for_service(std::chrono::milliseconds(10))) {
    RCLCPP_WARN(get_node()->get_logger(), "Configure maneuver service is not ready!");
    return;
  }

  configure_maneuver_client->async_send_request(
    request, [&](rclcpp::Client<ConfigureManeuver>::SharedFuture future) {
      auto response = future.get();
      if (response->maneuver.size() > 0) {
        change_maneuver(response->maneuver.front());
        callback(response->maneuver.front());
      } else {
        RCLCPP_WARN(get_node()->get_logger(), "Configure maneuver service response is empty!");
      }
    });
}

void ManeuverConsumer::configure_maneuver(
  const Maneuver & maneuver, const ManeuverCallback & callback)
{
  auto request = std::make_shared<ConfigureManeuver::Request>();
  request->maneuver.push_back(maneuver);

  request_to_configure_maneuver(request, callback);
}

std::shared_future<Maneuver> ManeuverConsumer::configure_maneuver(const Maneuver & maneuver)
{
  std::promise<Maneuver> promise;
  configure_maneuver(
    maneuver, [&](const Maneuver & maneuver) {
      promise.set_value(maneuver);
    });

  return promise.get_future();
}

void ManeuverConsumer::fetch_maneuver(const ManeuverCallback & callback)
{
  request_to_configure_maneuver(std::make_shared<ConfigureManeuver::Request>(), callback);
}

std::shared_future<Maneuver> ManeuverConsumer::fetch_maneuver()
{
  std::promise<Maneuver> promise;
  fetch_maneuver(
    [&](const Maneuver & maneuver) {
      promise.set_value(maneuver);
    });

  return promise.get_future();
}

void ManeuverConsumer::set_maneuver(const Maneuver & maneuver)
{
  maneuver_input_publisher->publish(maneuver);
}

rclcpp::Node::SharedPtr ManeuverConsumer::get_node() const
{
  return node;
}

const Maneuver & ManeuverConsumer::get_maneuver() const
{
  return current_maneuver;
}

void ManeuverConsumer::change_maneuver(const Maneuver & maneuver)
{
  if (on_change_maneuver) {
    on_change_maneuver(maneuver);
  }

  current_maneuver = maneuver;
}

}  // namespace tosshin_cpp

#endif  // TOSSHIN_CPP__CONSUMER__MANEUVER_CONSUMER_HPP_

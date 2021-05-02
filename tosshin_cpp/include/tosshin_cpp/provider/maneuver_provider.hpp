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

#ifndef TOSSHIN_CPP__PROVIDER__MANEUVER_PROVIDER_HPP_
#define TOSSHIN_CPP__PROVIDER__MANEUVER_PROVIDER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#include "../interfaces.hpp"

namespace tosshin_cpp
{

class ManeuverProvider
{
public:
  using OnConfigureManeuver = std::function<const Maneuver & (const Maneuver &)>;

  inline ManeuverProvider();

  inline explicit ManeuverProvider(
    rclcpp::Node::SharedPtr node, const std::string & prefix = NAVIGATION_PREFIX);

  inline void set_node(
    rclcpp::Node::SharedPtr node, const std::string & prefix = NAVIGATION_PREFIX);

  inline void set_on_configure_maneuver(const OnConfigureManeuver & callback);

  inline void set_maneuver(const Maneuver & maneuver);

  inline rclcpp::Node::SharedPtr get_node() const;

  inline const Maneuver & get_maneuver() const;

private:
  rclcpp::Node::SharedPtr node;

  rclcpp::Publisher<Maneuver>::SharedPtr maneuver_event_publisher;
  rclcpp::Subscription<Maneuver>::SharedPtr maneuver_input_subscription;

  rclcpp::Service<ConfigureManeuver>::SharedPtr configure_maneuver_service;

  OnConfigureManeuver on_configure_maneuver;

  Maneuver current_maneuver;
};

ManeuverProvider::ManeuverProvider()
{
}

ManeuverProvider::ManeuverProvider(rclcpp::Node::SharedPtr node, const std::string & prefix)
{
  set_node(node, prefix);
}

void ManeuverProvider::set_node(rclcpp::Node::SharedPtr node, const std::string & prefix)
{
  // Initialize the node
  this->node = node;

  // Initialize the maneuver event publisher
  {
    maneuver_event_publisher = get_node()->create_publisher<Maneuver>(
      prefix + MANEUVER_EVENT_SUFFIX, 10);

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Maneuver event publisher initialized on `" <<
        maneuver_event_publisher->get_topic_name() << "`!");
  }

  // Initialize the maneuver input subscription
  {
    maneuver_input_subscription = get_node()->create_subscription<Maneuver>(
      prefix + MANEUVER_INPUT_SUFFIX, 10,
      [this](const Maneuver::SharedPtr maneuver) {
        set_maneuver(*maneuver);
      });

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Maneuver input subscription initialized on `" <<
        maneuver_input_subscription->get_topic_name() << "`!");
  }

  // Initialize the configure maneuver service
  {
    configure_maneuver_service = get_node()->create_service<ConfigureManeuver>(
      prefix + CONFIGURE_MANEUVER_SUFFIX,
      [this](ConfigureManeuver::Request::SharedPtr request,
      ConfigureManeuver::Response::SharedPtr response) {
        if (request->maneuver.size() > 0) {
          set_maneuver(request->maneuver.front());
        }

        response->maneuver.push_back(get_maneuver());
      });

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Configure maneuver service initialized on `" <<
        configure_maneuver_service->get_service_name() << "`!");
  }
}

void ManeuverProvider::set_on_configure_maneuver(
  const ManeuverProvider::OnConfigureManeuver & callback)
{
  on_configure_maneuver = callback;
}

void ManeuverProvider::set_maneuver(const Maneuver & maneuver)
{
  if (on_configure_maneuver) {
    current_maneuver = on_configure_maneuver(maneuver);
  } else {
    current_maneuver = maneuver;
  }

  maneuver_event_publisher->publish(current_maneuver);
}

rclcpp::Node::SharedPtr ManeuverProvider::get_node() const
{
  return node;
}

const Maneuver & ManeuverProvider::get_maneuver() const
{
  return current_maneuver;
}

}  // namespace tosshin_cpp

#endif  // TOSSHIN_CPP__PROVIDER__MANEUVER_PROVIDER_HPP_

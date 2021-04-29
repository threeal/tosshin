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

#ifndef TOSSHIN_CPP__CONSUMER__ODOMETRY_CONSUMER_HPP_
#define TOSSHIN_CPP__CONSUMER__ODOMETRY_CONSUMER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "../utility.hpp"

namespace tosshin_cpp
{

class OdometryConsumer
{
public:
  using OnChangeOdometry = std::function<void (const Odometry &)>;

  inline OdometryConsumer();
  inline explicit OdometryConsumer(rclcpp::Node::SharedPtr node);

  inline void set_node(rclcpp::Node::SharedPtr node);

  inline void set_on_change_odometry(const OnChangeOdometry & callback);

  inline rclcpp::Node::SharedPtr get_node() const;

  inline const Odometry & get_odometry() const;

private:
  inline void change_odometry(const Odometry &);

  rclcpp::Node::SharedPtr node;

  rclcpp::Subscription<Odometry>::SharedPtr odometry_subscription;

  OnChangeOdometry on_change_odometry;

  Odometry current_odometry;
};

OdometryConsumer::OdometryConsumer()
{
}

OdometryConsumer::OdometryConsumer(rclcpp::Node::SharedPtr node)
{
  set_node(node);
}

void OdometryConsumer::set_node(rclcpp::Node::SharedPtr node)
{
  // Initialize the node
  this->node = node;

  // Initialize the odometry subscription
  {
    odometry_subscription = get_node()->create_subscription<Odometry>(
      "navigation/odometry", 10,
      [this](const Odometry::SharedPtr msg) {
        change_odometry(*msg);
      });

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Odometry subscription initialized on " <<
        odometry_subscription->get_topic_name() << "!");
  }
}

void OdometryConsumer::set_on_change_odometry(const OnChangeOdometry & callback)
{
  on_change_odometry = callback;
}

rclcpp::Node::SharedPtr OdometryConsumer::get_node() const
{
  return node;
}

const Odometry & OdometryConsumer::get_odometry() const
{
  return current_odometry;
}

void OdometryConsumer::change_odometry(const Odometry & odometry)
{
  if (on_change_odometry) {
    on_change_odometry(odometry);
  }

  current_odometry = odometry;
}

}  // namespace tosshin_cpp

#endif  // TOSSHIN_CPP__CONSUMER__ODOMETRY_CONSUMER_HPP_

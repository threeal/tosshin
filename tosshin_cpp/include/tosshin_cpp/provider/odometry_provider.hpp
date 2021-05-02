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

#ifndef TOSSHIN_CPP__PROVIDER__ODOMETRY_PROVIDER_HPP_
#define TOSSHIN_CPP__PROVIDER__ODOMETRY_PROVIDER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#include "../interface.hpp"

namespace tosshin_cpp
{

class OdometryProvider
{
public:
  inline OdometryProvider();

  inline explicit OdometryProvider(
    rclcpp::Node::SharedPtr node, const std::string & prefix = NAVIGATION_PREFIX);

  inline void set_node(
    rclcpp::Node::SharedPtr node, const std::string & prefix = NAVIGATION_PREFIX);

  inline void set_odometry(const Odometry & odometry);

  inline rclcpp::Node::SharedPtr get_node() const;

  inline const Odometry & get_odometry() const;

private:
  rclcpp::Node::SharedPtr node;

  rclcpp::Publisher<Odometry>::SharedPtr odometry_publisher;

  Odometry current_odometry;
};

OdometryProvider::OdometryProvider()
{
}

OdometryProvider::OdometryProvider(rclcpp::Node::SharedPtr node, const std::string & prefix)
{
  set_node(node, prefix);
}

void OdometryProvider::set_node(rclcpp::Node::SharedPtr node, const std::string & prefix)
{
  // Initialize the node
  this->node = node;

  // Initialize the odometry publisher
  {
    odometry_publisher = get_node()->create_publisher<Odometry>(prefix + ODOMETRY_SUFFIX, 10);

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Odometry publisher initialized on `" << odometry_publisher->get_topic_name() << "`!");
  }

  // Initial data publish
  set_odometry(get_odometry());
}

void OdometryProvider::set_odometry(const Odometry & odometry)
{
  current_odometry = odometry;
  odometry_publisher->publish(current_odometry);
}

rclcpp::Node::SharedPtr OdometryProvider::get_node() const
{
  return node;
}

const Odometry & OdometryProvider::get_odometry() const
{
  return current_odometry;
}

}  // namespace tosshin_cpp

#endif  // TOSSHIN_CPP__PROVIDER__ODOMETRY_PROVIDER_HPP_

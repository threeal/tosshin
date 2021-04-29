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

#include "./utility"

namespace tosshin_cpp
{

class OdometryProvider
{
public:
  inline OdometryProvider();

  inline explicit OdometryProvider(
    rclcpp::Node::SharedPtr node, const std::string & root_name = "/navigation");

  inline void set_node(rclcpp::Node::SharedPtr node, const std::string & root_name = "/navigation");

  inline void set_odometry(const Odometry & odometry);

  inline rclcpp::Node::SharedPtr get_node() const;

private:
  rclcpp::Node::SharedPtr node;

  rclcpp::Publisher<Odometry>::SharedPtr odometry_publisher;
};

OdometryProvider::OdometryProvider()
{
}

OdometryProvider::OdometryProvider(rclcpp::Node::SharedPtr node, const std::string & root_name)
{
  set_node(node, root_name);
}

void OdometryProvider::set_node(rclcpp::Node::SharedPtr node, const std::string & root_name)
{
  // Initialize the node
  this->node = node;

  // Initialize the odometry publisher
  {
    odometry_publisher = get_node()->create_publisher<Odometry>(root_name + "/odometry", 10);

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Odometry publisher initialized on " <<
        odometry_publisher->get_topic_name() << "!");
  }
}

void OdometryProvider::set_odometry(const Odometry & odometry)
{
  odometry_publisher->publish(odometry);
}

rclcpp::Node::SharedPtr OdometryProvider::get_node() const
{
  return node;
}

}  // namespace tosshin_cpp

#endif  // TOSSHIN_CPP__PROVIDER__ODOMETRY_PROVIDER_HPP_

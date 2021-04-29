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

#ifndef TOSSHIN_CPP__PROVIDER__NAVIGATION_PROVIDER_HPP_
#define TOSSHIN_CPP__PROVIDER__NAVIGATION_PROVIDER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "./maneuver_provider.hpp"
#include "./odometry_provider.hpp"

namespace tosshin_cpp
{

class NavigationProvider : public OdometryProvider, public ManeuverProvider
{
public:
  inline NavigationProvider();
  inline explicit NavigationProvider(rclcpp::Node::SharedPtr node);

  inline void set_node(rclcpp::Node::SharedPtr node);

  inline rclcpp::Node::SharedPtr get_node() const;

private:
  rclcpp::Node::SharedPtr node;
};

NavigationProvider::NavigationProvider()
{
}

NavigationProvider::NavigationProvider(rclcpp::Node::SharedPtr node)
{
  set_node(node);
}

void NavigationProvider::set_node(rclcpp::Node::SharedPtr node)
{
  // Initialize the node
  this->node = node;

  // Set parents's node
  OdometryProvider::set_node(get_node());
  ManeuverProvider::set_node(get_node());
}

rclcpp::Node::SharedPtr NavigationProvider::get_node() const
{
  return node;
}

}  // namespace tosshin_cpp

#endif  // TOSSHIN_CPP__PROVIDER__NAVIGATION_PROVIDER_HPP_

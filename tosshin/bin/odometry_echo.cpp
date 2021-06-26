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

#include <argparse/argparse.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tosshin/tosshin.hpp>

#include <memory>
#include <string>

namespace tsn = tosshin;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto program = argparse::ArgumentParser("odometry_echo", "0.3.0");

  program.add_argument("--odometry-topic")
  .help("name of the odometry topic")
  .default_value(std::string("/odom"));

  std::string odometry_topic;

  // Try to parse arguments
  try {
    program.parse_args(argc, argv);

    odometry_topic = program.get<std::string>("--odometry-topic");
  } catch (const std::exception & e) {
    std::cout << e.what() << std::endl;
    std::cout << program;
    return 1;
  }

  auto node = std::make_shared<rclcpp::Node>("odometry_echo");

  auto odometry_subscription = node->create_subscription<tsn::msg::Odometry>(
    odometry_topic, 10,
    [node](const tsn::msg::Odometry::SharedPtr msg) {
      auto & position = msg->pose.pose.position;
      auto & linear = msg->twist.twist.linear;
      auto & angular = msg->twist.twist.angular;
      auto orientation = tsn::extract_quaternion(msg->pose.pose.orientation).euler();

      RCLCPP_INFO_STREAM(
        node->get_logger(),
        std::fixed << std::setprecision(2) <<
          "\nposition\t: " << position.x << ", " << position.y << ", " << position.z <<
          "\norientation\t: " <<
          orientation.roll.degree() << ", " <<
          orientation.pitch.degree() << ", " <<
          orientation.yaw.degree() <<
          "\nlinear vel\t: " << linear.x << ", " << linear.y << ", " << linear.z <<
          "\nangular vel\t: " << angular.x << ", " << angular.y << ", " << angular.z);
    });

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}

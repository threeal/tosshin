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

#include <tosshin_cpp/tosshin_cpp.hpp>
#include <keisan/keisan.hpp>
#include <rclcpp/rclcpp.hpp>

#include <iomanip>
#include <memory>
#include <string>

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<std::string> navigation_prefix;

  // Handle arguments
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      std::cout << "Usage: ros2 run tosshin_cpp navigation_provider_dummy " <<
        "[options] [navigation_prefix]" <<
        "\n\nPositional arguments:" <<
        "\nnavigation_prefix\tprefix name for navigation's topics and services" <<
        "\n\nOptional arguments:" <<
        "\n-h, --help\tshow this help message and exit";

      return 1;
    } else if (navigation_prefix == nullptr) {
      navigation_prefix = std::make_shared<std::string>(arg);
    }
  }

  auto node = std::make_shared<rclcpp::Node>("navigation_provider_dummy");

  std::shared_ptr<tosshin_cpp::NavigationProvider> navigation_provider;
  if (navigation_prefix != nullptr) {
    navigation_provider = std::make_shared<tosshin_cpp::NavigationProvider>(
      node, *navigation_prefix);
  } else {
    navigation_provider = std::make_shared<tosshin_cpp::NavigationProvider>(node);
  }

  RCLCPP_INFO(node->get_logger(), "Press enter to continue");
  std::cin.get();

  auto update_timer = node->create_wall_timer(
    100ms, [&]() {
      // Change odometry according to the maneuver
      {
        auto odometry = navigation_provider->get_odometry();
        auto maneuver = navigation_provider->get_maneuver();

        // Odometry orientation is in degree while maneuver yaw is in radian per minute
        odometry.orientation.yaw = keisan::wrap_deg(
          odometry.orientation.yaw + keisan::rad_to_deg(maneuver.yaw / 600));

        auto angle = keisan::deg_to_rad(odometry.orientation.yaw);

        // Odometry position is in meter while maneuver forward and left are in meter per minute
        odometry.position.x += (maneuver.forward * cos(angle) - maneuver.left * sin(angle)) / 600;
        odometry.position.y += (maneuver.forward * sin(angle) + maneuver.left * cos(angle)) / 600;

        navigation_provider->set_odometry(odometry);
      }

      // Clear screen
      std::cout << "\033[2J\033[2H" << std::endl;

      auto odometry = navigation_provider->get_odometry();
      auto maneuver = navigation_provider->get_maneuver();

      // Print navigation info
      RCLCPP_INFO_STREAM(
        node->get_logger(),
        std::fixed << std::setprecision(1) <<
          "\nOdometry:" <<
          "\n- Position\t: " << odometry.position.x << ", " << odometry.position.y <<
          "\n- Orientation\t: " << odometry.orientation.yaw <<
          "\n\nManeuver:" <<
          "\n- Forward\t: " << maneuver.forward <<
          "\n- Left\t\t: " << maneuver.left <<
          "\n- Yaw\t\t: " << maneuver.yaw);
    });

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}

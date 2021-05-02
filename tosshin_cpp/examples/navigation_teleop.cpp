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

#include <termios.h>
#include <unistd.h>

#include <iomanip>
#include <memory>
#include <string>

#define STDIN 0

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<std::string> navigation_prefix;

  // Handle arguments
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      std::cout << "Usage: ros2 run tosshin_cpp navigation_teleop [options] [navigation_prefix]" <<
        "\n\nPositional arguments:" <<
        "\nnavigation_prefix\tprefix name for navigation's topics and services" <<
        "\n\nOptional arguments:" <<
        "\n-h, --help\t\tshow this help message and exit" << std::endl;

      return 1;
    } else if (navigation_prefix == nullptr) {
      navigation_prefix = std::make_shared<std::string>(arg);
    }
  }

  auto node = std::make_shared<rclcpp::Node>("navigation_consumer_teleop");

  std::shared_ptr<tosshin_cpp::NavigationConsumer> navigation_consumer;
  if (navigation_prefix != nullptr) {
    navigation_consumer = std::make_shared<tosshin_cpp::NavigationConsumer>(
      node, *navigation_prefix);
  } else {
    navigation_consumer = std::make_shared<tosshin_cpp::NavigationConsumer>(node);
  }

  // Obtain the original terminal configuration
  struct termios original_term;
  tcgetattr(STDIN, &original_term);

  // Create a new non-blocking terminal configuration
  struct termios nonblock_term = original_term;
  nonblock_term.c_lflag &= ~ICANON;
  nonblock_term.c_lflag &= ~ECHO;
  nonblock_term.c_lflag &= ~ISIG;
  nonblock_term.c_cc[VMIN] = 0;
  nonblock_term.c_cc[VTIME] = 0;

  RCLCPP_INFO(node->get_logger(), "Press enter to continue");
  std::cin.get();

  auto update_timer = node->create_wall_timer(
    100ms, [&]() {
      // Modify the terminal configuration
      tcsetattr(STDIN, TCSANOW, &nonblock_term);

      // Handle keyboard input
      char input;
      while (read(STDIN, &input, 1) > 0) {
        auto maneuver = navigation_consumer->get_maneuver();

        switch (toupper(input)) {
          case 'W':
            if (maneuver.forward < 0.0) {
              maneuver.forward = 0.0;
              maneuver.left = 0.0;
              maneuver.yaw = 0.0;
              break;
            }

            maneuver.forward += 10.0;
            break;

          case 'S':
            if (maneuver.forward > 0.0) {
              maneuver.forward = 0.0;
              maneuver.left = 0.0;
              maneuver.yaw = 0.0;
              break;
            }

            maneuver.forward -= 10.0;
            break;

          case 'A':
            if (maneuver.left < 0.0) {
              maneuver.left = 0.0;
              break;
            }

            maneuver.left += 10.0;
            break;

          case 'D':
            if (maneuver.left > 0.0) {
              maneuver.left = 0.0;
              break;
            }

            maneuver.left -= 10.0;
            break;

          case 'Q':
            if (maneuver.yaw < 0.0) {
              maneuver.yaw = 0.0;
              break;
            }

            maneuver.yaw += 10.0;
            break;

          case 'E':
            if (maneuver.yaw > 0.0) {
              maneuver.yaw = 0.0;
              break;
            }

            maneuver.yaw -= 10.0;
            break;
        }

        navigation_consumer->set_maneuver(maneuver);
      }

      // Restore the terminal configuration
      tcsetattr(STDIN, TCSANOW, &original_term);

      // Clear screen
      std::cout << "\033[2J\033[2H" << std::endl;

      auto odometry = navigation_consumer->get_odometry();
      auto maneuver = navigation_consumer->get_maneuver();

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
          "\n- Yaw\t\t: " << maneuver.yaw <<
          "\n\nW/S -> forward maneuver\tA/D -> left maneuver\tQ/E -> yaw maneuver");
    });

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}

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

#include <gtest/gtest.h>
#include <tosshin/tosshin.hpp>

TEST(InterfacesTest, OdometryTest)
{
  tosshin::msg::Odometry odometry;
  {
    odometry.header.stamp.sec = 0;
    odometry.header.stamp.nanosec = 0;
    odometry.header.frame_id = "odom";
    odometry.child_frame_id = "base_footprint";

    tosshin::msg::PoseWithCovariance pose_with_covariance;
    {
      tosshin::msg::Pose pose;
      {
        tosshin::msg::Point position;
        {
          position.x = 0.0;
          position.y = 0.0;
          position.z = 0.0;
        }

        tosshin::msg::Quaternion orientation;
        {
          orientation.x = 0.0;
          orientation.y = 0.0;
          orientation.z = 0.0;
          orientation.w = 0.0;
        }

        pose.position = position;
        pose.orientation = orientation;
      }

      pose_with_covariance.pose = pose;

      for (size_t i = 0; i < 36; ++i) {
        pose_with_covariance.covariance[i] = 0.0;
      }
    }

    tosshin::msg::TwistWithCovariance twist_with_covariance;
    {
      tosshin::msg::Twist twist;
      {
        tosshin::msg::Vector3 linear;
        {
          linear.x = 0.0;
          linear.y = 0.0;
          linear.z = 0.0;
        }

        tosshin::msg::Vector3 angular;
        {
          angular.x = 0.0;
          angular.y = 0.0;
          angular.z = 0.0;
        }

        twist.linear = linear;
        twist.angular = angular;
      }

      twist_with_covariance.twist = twist;

      for (size_t i = 0; i < 36; ++i) {
        twist_with_covariance.covariance[i] = 0.0;
      }
    }

    odometry.pose = pose_with_covariance;
    odometry.twist = twist_with_covariance;
  }
}

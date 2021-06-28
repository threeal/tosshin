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
#include <keisan/keisan.hpp>
#include <tosshin/tosshin.hpp>

namespace ksn = keisan;
namespace tsn = tosshin;

TEST(ConversionTest, MakePointFromKeisanPoint)
{
  ksn::Point3 point(1.0, 2.0, 3.0);
  auto msg = tsn::make_point(point);

  ASSERT_DOUBLE_EQ(msg.x, point.x);
  ASSERT_DOUBLE_EQ(msg.y, point.y);
  ASSERT_DOUBLE_EQ(msg.z, point.z);
}

TEST(ConversionTest, MakePointFromVector3)
{
  tsn::msg::Vector3 vector3;

  vector3.x = 1.0;
  vector3.y = 2.0;
  vector3.z = 3.0;

  auto point = tsn::make_point(vector3);

  ASSERT_DOUBLE_EQ(point.x, vector3.x);
  ASSERT_DOUBLE_EQ(point.y, vector3.y);
  ASSERT_DOUBLE_EQ(point.z, vector3.z);
}

TEST(ConversionTest, ExtractPoint)
{
  tsn::msg::Point msg;

  msg.x = 1.0;
  msg.y = 2.0;
  msg.z = 3.0;

  auto point = tsn::extract_point(msg);

  ASSERT_DOUBLE_EQ(point.x, msg.x);
  ASSERT_DOUBLE_EQ(point.y, msg.y);
  ASSERT_DOUBLE_EQ(point.z, msg.z);
}

TEST(ConversionTest, MakePointXY)
{
  ksn::Point2 point(1.0, 2.0);
  auto msg = tsn::make_point_xy(point);

  ASSERT_DOUBLE_EQ(msg.x, point.x);
  ASSERT_DOUBLE_EQ(msg.y, point.y);
  ASSERT_DOUBLE_EQ(msg.z, 0.0);
}

TEST(ConversionTest, ExtractPointXY)
{
  tsn::msg::Point msg;

  msg.x = 1.0;
  msg.y = 2.0;

  auto point = tsn::extract_point_xy(msg);

  ASSERT_DOUBLE_EQ(point.x, msg.x);
  ASSERT_DOUBLE_EQ(point.y, msg.y);
}

TEST(ConversionTest, MakeQuaternion)
{
  ksn::Quaternion quaternion(1.0, 2.0, 3.0, 4.0);
  auto msg = tsn::make_quaternion(quaternion);

  ASSERT_DOUBLE_EQ(msg.x, quaternion.x);
  ASSERT_DOUBLE_EQ(msg.y, quaternion.y);
  ASSERT_DOUBLE_EQ(msg.z, quaternion.z);
  ASSERT_DOUBLE_EQ(msg.w, quaternion.w);
}

TEST(ConversionTest, ExtractQuaternion)
{
  tsn::msg::Quaternion msg;

  msg.x = 1.0;
  msg.y = 2.0;
  msg.z = 3.0;
  msg.w = 4.0;

  auto quaternion = tsn::extract_quaternion(msg);

  ASSERT_DOUBLE_EQ(quaternion.x, msg.x);
  ASSERT_DOUBLE_EQ(quaternion.y, msg.y);
  ASSERT_DOUBLE_EQ(quaternion.z, msg.z);
  ASSERT_DOUBLE_EQ(quaternion.w, msg.w);
}

TEST(ConversionTest, MakeVector3FromKeisanPoint)
{
  ksn::Point3 point(1.0, 2.0, 3.0);
  auto msg = tsn::make_vector3(point);

  ASSERT_DOUBLE_EQ(msg.x, point.x);
  ASSERT_DOUBLE_EQ(msg.y, point.y);
  ASSERT_DOUBLE_EQ(msg.z, point.z);
}

TEST(ConversionTest, MakeVector3FromPoint)
{
  tsn::msg::Point point;

  point.x = 1.0;
  point.y = 2.0;
  point.z = 3.0;

  auto vector3 = tsn::make_vector3(point);

  ASSERT_DOUBLE_EQ(vector3.x, point.x);
  ASSERT_DOUBLE_EQ(vector3.y, point.y);
  ASSERT_DOUBLE_EQ(vector3.z, point.z);
}

TEST(ConversionTest, ExtractVector3)
{
  tsn::msg::Vector3 msg;

  msg.x = 1.0;
  msg.y = 2.0;
  msg.z = 3.0;

  auto point = tsn::extract_vector3(msg);

  ASSERT_DOUBLE_EQ(point.x, msg.x);
  ASSERT_DOUBLE_EQ(point.y, msg.y);
  ASSERT_DOUBLE_EQ(point.z, msg.z);
}

TEST(ConversionTest, MakeVector3XY)
{
  ksn::Point2 point(1.0, 2.0);
  auto msg = tsn::make_vector3_xy(point);

  ASSERT_DOUBLE_EQ(msg.x, point.x);
  ASSERT_DOUBLE_EQ(msg.y, point.y);
  ASSERT_DOUBLE_EQ(msg.z, 0.0);
}

TEST(ConversionTest, ExtractVector3XY)
{
  tsn::msg::Vector3 msg;

  msg.x = 1.0;
  msg.y = 2.0;

  auto point = tsn::extract_vector3_xy(msg);

  ASSERT_DOUBLE_EQ(point.x, msg.x);
  ASSERT_DOUBLE_EQ(point.y, msg.y);
}

TEST(ConversionTest, MakeOdometryFromTransformStamped)
{
  tsn::msg::TransformStamped transform_stamped;

  transform_stamped.header.stamp.sec = 10;
  transform_stamped.header.stamp.nanosec = 100;
  transform_stamped.header.frame_id = "foo";
  transform_stamped.child_frame_id = "goo";

  transform_stamped.transform.translation = tsn::make_vector3({1.0, 2.0, 3.0});
  transform_stamped.transform.rotation = tsn::make_quaternion({4.0, 5.0, 6.0, 7.0});

  auto odometry = tsn::make_odometry(transform_stamped);

  ASSERT_EQ(odometry.header, transform_stamped.header);
  ASSERT_EQ(odometry.child_frame_id, transform_stamped.child_frame_id);

  const auto & position = odometry.pose.pose.position;
  const auto & translation = transform_stamped.transform.translation;

  ASSERT_DOUBLE_EQ(position.x, translation.x);
  ASSERT_DOUBLE_EQ(position.y, translation.y);
  ASSERT_DOUBLE_EQ(position.z, translation.z);

  const auto & orientation = odometry.pose.pose.orientation;
  const auto & rotation = transform_stamped.transform.rotation;

  ASSERT_DOUBLE_EQ(orientation.x, rotation.x);
  ASSERT_DOUBLE_EQ(orientation.y, rotation.y);
  ASSERT_DOUBLE_EQ(orientation.z, rotation.z);
  ASSERT_DOUBLE_EQ(orientation.w, rotation.w);
}

TEST(ConversionTest, MakeTransformStampedFromOdometry)
{
  tsn::msg::Odometry odometry;

  odometry.header.stamp.sec = 10;
  odometry.header.stamp.nanosec = 100;
  odometry.header.frame_id = "foo";
  odometry.child_frame_id = "goo";

  odometry.pose.pose.position = tsn::make_point({1.0, 2.0, 3.0});
  odometry.pose.pose.orientation = tsn::make_quaternion({4.0, 5.0, 6.0, 7.0});

  auto transform_stamped = tsn::make_transform_stamped(odometry);

  ASSERT_EQ(transform_stamped.header, odometry.header);
  ASSERT_EQ(transform_stamped.child_frame_id, odometry.child_frame_id);

  const auto & translation = transform_stamped.transform.translation;
  const auto & position = odometry.pose.pose.position;

  ASSERT_DOUBLE_EQ(translation.x, position.x);
  ASSERT_DOUBLE_EQ(translation.y, position.y);
  ASSERT_DOUBLE_EQ(translation.z, position.z);

  const auto & rotation = transform_stamped.transform.rotation;
  const auto & orientation = odometry.pose.pose.orientation;

  ASSERT_DOUBLE_EQ(rotation.x, orientation.x);
  ASSERT_DOUBLE_EQ(rotation.y, orientation.y);
  ASSERT_DOUBLE_EQ(rotation.z, orientation.z);
  ASSERT_DOUBLE_EQ(rotation.w, orientation.w);
}

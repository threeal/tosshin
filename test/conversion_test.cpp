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

TEST(ConversionTest, MakePoint)
{
  ksn::Point3 point(1.0, 2.0, 3.0);
  auto msg = tsn::make_point(point);

  ASSERT_DOUBLE_EQ(msg.x, point.x);
  ASSERT_DOUBLE_EQ(msg.y, point.y);
  ASSERT_DOUBLE_EQ(msg.z, point.z);
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

TEST(ConversionTest, MakeVector3)
{
  ksn::Point3 point(1.0, 2.0, 3.0);
  auto msg = tsn::make_vector3(point);

  ASSERT_DOUBLE_EQ(msg.x, point.x);
  ASSERT_DOUBLE_EQ(msg.y, point.y);
  ASSERT_DOUBLE_EQ(msg.z, point.z);
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

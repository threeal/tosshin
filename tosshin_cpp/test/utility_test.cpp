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
#include <tosshin_cpp/tosshin_cpp.hpp>

TEST(UtilityTest, PositionToPoint) {
  tosshin_cpp::Position position;

  position.x = 3.5;
  position.y = 7.8;

  auto point = tosshin_cpp::position_to_point(position);

  ASSERT_DOUBLE_EQ(point.x, position.x);
  ASSERT_DOUBLE_EQ(point.y, position.y);
}

TEST(UtilityTest, PointToPosition) {
  keisan::Point2 point(3.5, 7.8);

  auto position = tosshin_cpp::point_to_position(point);

  ASSERT_DOUBLE_EQ(position.x, point.x);
  ASSERT_DOUBLE_EQ(position.y, point.y);
}

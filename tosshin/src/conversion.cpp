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

#include <tosshin/conversion.hpp>

namespace tosshin
{

msg::Point make_point(const keisan::Point3 & point)
{
  msg::Point msg;

  msg.x = point.x;
  msg.y = point.y;
  msg.z = point.z;

  return msg;
}

keisan::Point3 extract_point(const msg::Point & msg)
{
  return keisan::Point3(msg.x, msg.y, msg.z);
}

msg::Point make_point_xy(const keisan::Point2 & point)
{
  msg::Point msg;

  msg.x = point.x;
  msg.y = point.y;
  msg.z = 0.0;

  return msg;
}

keisan::Point2 extract_point_xy(const msg::Point & msg)
{
  return keisan::Point2(msg.x, msg.y);
}

msg::Quaternion make_quaternion(const keisan::Quaternion & quaternion)
{
  msg::Quaternion msg;

  msg.x = quaternion.x;
  msg.y = quaternion.y;
  msg.z = quaternion.z;
  msg.w = quaternion.w;

  return msg;
}

keisan::Quaternion extract_quaternion(const msg::Quaternion & msg)
{
  return keisan::Quaternion(msg.x, msg.y, msg.z, msg.w);
}

msg::Vector3 make_vector3(const keisan::Point3 & point)
{
  msg::Vector3 msg;

  msg.x = point.x;
  msg.y = point.y;
  msg.z = point.z;

  return msg;
}

keisan::Point3 extract_vector3(const msg::Vector3 & msg)
{
  return keisan::Point3(msg.x, msg.y, msg.z);
}

msg::Vector3 make_vector3_xy(const keisan::Point2 & point)
{
  msg::Vector3 msg;

  msg.x = point.x;
  msg.y = point.y;
  msg.z = 0.0;

  return msg;
}

keisan::Point2 extract_vector3_xy(const msg::Vector3 & msg)
{
  return keisan::Point2(msg.x, msg.y);
}

}  // namespace tosshin

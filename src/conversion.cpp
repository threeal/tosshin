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

namespace ksn = keisan;

namespace tosshin
{

msg::Point make_point(const ksn::Point3 & point)
{
  msg::Point msg;

  msg.x = point.x;
  msg.y = point.y;
  msg.z = point.z;

  return msg;
}

msg::Point make_point(const msg::Vector3 & vector3)
{
  msg::Point point;

  point.x = vector3.x;
  point.y = vector3.y;
  point.z = vector3.z;

  return point;
}

ksn::Point3 extract_point(const msg::Point & msg)
{
  return ksn::Point3(msg.x, msg.y, msg.z);
}

msg::Point make_point_xy(const ksn::Point2 & point)
{
  msg::Point msg;

  msg.x = point.x;
  msg.y = point.y;
  msg.z = 0.0;

  return msg;
}

ksn::Point2 extract_point_xy(const msg::Point & msg)
{
  return ksn::Point2(msg.x, msg.y);
}

msg::Quaternion make_quaternion(const ksn::Quaternion & quaternion)
{
  msg::Quaternion msg;

  msg.x = quaternion.x;
  msg.y = quaternion.y;
  msg.z = quaternion.z;
  msg.w = quaternion.w;

  return msg;
}

ksn::Quaternion extract_quaternion(const msg::Quaternion & msg)
{
  return ksn::Quaternion(msg.x, msg.y, msg.z, msg.w);
}

msg::Vector3 make_vector3(const ksn::Point3 & point)
{
  msg::Vector3 msg;

  msg.x = point.x;
  msg.y = point.y;
  msg.z = point.z;

  return msg;
}

msg::Vector3 make_vector3(const msg::Point & point)
{
  msg::Vector3 vector3;

  vector3.x = point.x;
  vector3.y = point.y;
  vector3.z = point.z;

  return vector3;
}

ksn::Point3 extract_vector3(const msg::Vector3 & msg)
{
  return ksn::Point3(msg.x, msg.y, msg.z);
}

msg::Vector3 make_vector3_xy(const ksn::Point2 & point)
{
  msg::Vector3 msg;

  msg.x = point.x;
  msg.y = point.y;
  msg.z = 0.0;

  return msg;
}

ksn::Point2 extract_vector3_xy(const msg::Vector3 & msg)
{
  return ksn::Point2(msg.x, msg.y);
}

msg::Odometry make_odometry(const msg::TransformStamped & transform_stamped)
{
  msg::Odometry odometry;

  odometry.header = transform_stamped.header;
  odometry.child_frame_id = transform_stamped.child_frame_id;

  auto & position = odometry.pose.pose.position;
  auto & translation = transform_stamped.transform.translation;

  position.x = translation.x;
  position.y = translation.y;
  position.z = translation.z;

  odometry.pose.pose.position = make_point(transform_stamped.transform.translation);
  odometry.pose.pose.orientation = transform_stamped.transform.rotation;

  return odometry;
}

msg::TransformStamped make_transform_stamped(const msg::Odometry & odometry)
{
  msg::TransformStamped transform_stamped;

  transform_stamped.header = odometry.header;
  transform_stamped.child_frame_id = odometry.child_frame_id;
  transform_stamped.transform.translation = make_vector3(odometry.pose.pose.position);
  transform_stamped.transform.rotation = odometry.pose.pose.orientation;

  return transform_stamped;
}

}  // namespace tosshin

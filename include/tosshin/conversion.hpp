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

#ifndef TOSSHIN__CONVERSION_HPP_
#define TOSSHIN__CONVERSION_HPP_

#include <keisan/keisan.hpp>

#include "./interfaces.hpp"

namespace tosshin
{

msg::Point make_point(const keisan::Point3 & point);
msg::Point make_point(const msg::Vector3 & vector3);
keisan::Point3 extract_point(const msg::Point & msg);

msg::Point make_point_xy(const keisan::Point2 & point);
keisan::Point2 extract_point_xy(const msg::Point & msg);

msg::Quaternion make_quaternion(const keisan::Quaternion & quaternion);
keisan::Quaternion extract_quaternion(const msg::Quaternion & msg);

msg::Vector3 make_vector3(const keisan::Point3 & point);
msg::Vector3 make_vector3(const msg::Point & point);
keisan::Point3 extract_vector3(const msg::Vector3 & msg);

msg::Vector3 make_vector3_xy(const keisan::Point2 & point);
keisan::Point2 extract_vector3_xy(const msg::Vector3 & msg);

msg::Odometry make_odometry(const msg::TransformStamped & transform_stamped);
msg::TransformStamped make_transform_stamped(const msg::Odometry & odometry);

}  // namespace tosshin

#endif  // TOSSHIN__CONVERSION_HPP_

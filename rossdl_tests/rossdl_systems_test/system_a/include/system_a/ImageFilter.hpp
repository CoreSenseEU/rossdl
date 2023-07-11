// Copyright 2023 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef SYSTEM_A__IMAGEFILTER_HPP_
#define SYSTEM_A__IMAGEFILTER_HPP_

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

#include "system_a/Nodes.hpp"

#include "rclcpp/rclcpp.hpp"

namespace system_a
{

class ImageFilter : public ImageFilterBase
{
public:
  explicit ImageFilter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  virtual void image_in_callback(sensor_msgs::msg::Image::SharedPtr msg);
  virtual void laser_in_callback(sensor_msgs::msg::LaserScan::SharedPtr msg);
};

}  // namespace system_a

#endif  // SYSTEM_A__IMAGEFILTER_HPP_

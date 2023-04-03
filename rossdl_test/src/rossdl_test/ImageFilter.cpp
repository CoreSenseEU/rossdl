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


#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

#include "rossdl_test/ImageFilter.hpp"

#include "rclcpp/rclcpp.hpp"

namespace rossdl_test
{

ImageFilter::ImageFilter()
{
}

void
ImageFilter::image_sub_3_callback(sensor_msgs::msg::Image::SharedPtr msg)
{
  (void)msg;
  RCLCPP_INFO(get_logger(), "Message received");

  auto pub = get_publisher<sensor_msgs::msg::Image>("image_out");
  if (pub != nullptr) {
    pub->publish(*msg);
  } else {
    RCLCPP_ERROR(get_logger(), "Error getting image_out publisher");
  }
}

}  // namespace rossdl_test
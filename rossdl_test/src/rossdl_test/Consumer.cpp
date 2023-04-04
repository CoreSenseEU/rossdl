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

#include "rossdl_test/Consumer.hpp"

#include "rclcpp/rclcpp.hpp"

namespace rossdl_test
{

Consumer::Consumer()
{
}

void
Consumer::image_in_callback(sensor_msgs::msg::Image::SharedPtr msg)
{
  (void)msg;
  RCLCPP_INFO(get_logger(), "Image message received");
}

void
Consumer::description_in_callback(std_msgs::msg::String::SharedPtr msg)
{
  (void)msg;
  RCLCPP_INFO(get_logger(), "String message received");
}

}  // namespace rossdl_test

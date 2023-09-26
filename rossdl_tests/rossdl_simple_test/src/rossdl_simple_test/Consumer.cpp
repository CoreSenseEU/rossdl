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

#include "rossdl_simple_test/Consumer.hpp"

#include "rclcpp/rclcpp.hpp"

namespace rossdl_simple_test
{

Consumer::Consumer(const rclcpp::NodeOptions & options)
: ConsumerBase(options)
{
}

void
Consumer::image_in_callback(sensor_msgs::msg::Image::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Image message received");
  auto pub = get_publisher<sensor_msgs::msg::Image>("image_out");
  pub->publish(*msg);
}

void
Consumer::description_in_callback(std_msgs::msg::String::SharedPtr msg)
{
  (void)msg;
  RCLCPP_INFO(get_logger(), "String message received");
}

}  // namespace rossdl_simple_test

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rossdl_simple_test::Consumer)

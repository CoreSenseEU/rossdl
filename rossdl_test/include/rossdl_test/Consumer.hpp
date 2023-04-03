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


#ifndef ROSSDL_TEST__CONSUMER_HPP_
#define ROSSDL_TEST__CONSUMER_HPP_

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

#include "rossdl_test/Nodes2.hpp"

#include "rclcpp/rclcpp.hpp"

namespace rossdl_test
{

class Consumer : public ConsumerBase2
{
public:
  Consumer();

protected:
  virtual void image_sub_0_callback(sensor_msgs::msg::Image::SharedPtr msg);
  virtual void string_sub_1_callback(std_msgs::msg::String::SharedPtr msg);
};

}  // namespace rossdl_test

#endif  // ROSSDL_TEST__CONSUMER_HPP_

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


#include <memory>

#include "rossdl_test/ImageFilter.hpp"
#include "rossdl_test/Consumer.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  auto image_filter = std::make_shared<rossdl_test::ImageFilter>();
  auto consumer = std::make_shared<rossdl_test::Consumer>();

  exe.add_node(image_filter);
  exe.add_node(consumer);

  exe.spin();

  rclcpp::shutdown();

  return 0;
}

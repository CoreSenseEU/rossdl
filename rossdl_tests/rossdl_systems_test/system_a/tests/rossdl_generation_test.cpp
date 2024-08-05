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


#include <string>
#include <memory>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "system_a/ImageFilter.hpp"

#include "gtest/gtest.h"


TEST(rossdl_generation_test, image_filter_unit)
{
  auto image_filter = std::make_shared<system_a::ImageFilter>();

  // Parameters
  ASSERT_TRUE(image_filter->has_parameter("description_label"));
  auto param1 = image_filter->get_parameter("description_label");
  ASSERT_EQ(param1.get_type(), rclcpp::ParameterType::PARAMETER_STRING);

  auto topics = image_filter->get_topic_names_and_types();

  ASSERT_EQ(topics.size(), 6u);
  ASSERT_NE(topics.find("/image_filter/image_out"), topics.end());
  ASSERT_NE(topics.find("/image_filter/description_out"), topics.end());
  ASSERT_NE(topics.find("/image_filter/image_in"), topics.end());
  ASSERT_NE(topics.find("/image_filter/laser_in"), topics.end());
  ASSERT_NE(topics.find("/rosout"), topics.end());
  ASSERT_NE(topics.find("/parameter_events"), topics.end());

  ASSERT_EQ(topics["/image_filter/image_in"].size(), 1u);
  ASSERT_EQ(topics["/image_filter/image_in"][0], "sensor_msgs/msg/Image");
  ASSERT_EQ(topics["/image_filter/image_out"].size(), 1u);
  ASSERT_EQ(topics["/image_filter/image_out"][0], "sensor_msgs/msg/Image");
  ASSERT_EQ(topics["/image_filter/description_out"].size(), 1u);
  ASSERT_EQ(topics["/image_filter/description_out"][0], "std_msgs/msg/String");

  {
    auto info = image_filter->get_subscriptions_info_by_topic("/image_filter/image_in");
    ASSERT_EQ(info.size(), 1u);
    ASSERT_EQ(info[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::BestEffort);
    ASSERT_EQ(info[0].qos_profile().liveliness(), rclcpp::LivelinessPolicy::Automatic);
    ASSERT_EQ(info[0].qos_profile().durability(), rclcpp::DurabilityPolicy::Volatile);
    ASSERT_EQ(info[0].qos_profile().depth(), 5u);
  }
  {
    auto info = image_filter->get_publishers_info_by_topic("/image_filter/image_out");
    ASSERT_EQ(info.size(), 1u);
    ASSERT_EQ(info[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
    ASSERT_EQ(info[0].qos_profile().liveliness(), rclcpp::LivelinessPolicy::Automatic);
    ASSERT_EQ(info[0].qos_profile().durability(), rclcpp::DurabilityPolicy::Volatile);
    ASSERT_EQ(info[0].qos_profile().depth(), 5u);
  }
  {
    auto info = image_filter->get_publishers_info_by_topic("/image_filter/description_out");
    ASSERT_EQ(info.size(), 1u);
    ASSERT_EQ(info[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
    ASSERT_EQ(info[0].qos_profile().liveliness(), rclcpp::LivelinessPolicy::Automatic);
    ASSERT_EQ(info[0].qos_profile().durability(), rclcpp::DurabilityPolicy::Volatile);
    ASSERT_EQ(info[0].qos_profile().depth(), 100u);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}

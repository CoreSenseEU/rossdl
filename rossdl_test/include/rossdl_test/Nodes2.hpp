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

#ifndef ROSSDL_TEST__NODES2_HPP_
#define ROSSDL_TEST__NODES2_HPP_

#include <string>
#include <optional>
#include <typeinfo>

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

#include "rclcpp/rclcpp.hpp"

namespace rossdl_test
{

class ImageFilterBase2 : public rclcpp::Node
{
public:
  ImageFilterBase2();

protected:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_0_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_1_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_3_;

  virtual void image_sub_3_callback(sensor_msgs::msg::Image::SharedPtr msg) = 0;

  template<typename T>
  typename rclcpp::Publisher<T>::SharedPtr
  get_publisher(const std::string & id)
  {
    auto ret_pub = std::dynamic_pointer_cast<typename rclcpp::Publisher<T>>(
      get_publisher_base(id));
    return ret_pub;
  }

  template<typename T>
  typename rclcpp::Subscription<T>::SharedPtr
  get_subscription(const std::string & id)
  {
    auto ret_sub = std::dynamic_pointer_cast<typename rclcpp::Subscription<T>>(
      get_subscription_base(id));
    return ret_sub;
  }

  typename rclcpp::PublisherBase::SharedPtr
  get_publisher_base(const std::string & id)
  {
    if (id == "image_out") {
      return image_pub_0_;
    } else if (id == "description_out") {
      return string_pub_1_;
    } else {
      RCLCPP_ERROR(get_logger(), "Publisher [%s] not found", id.c_str());
      return nullptr;
    }
  }

  typename rclcpp::SubscriptionBase::SharedPtr
  get_subscription_base(const std::string & id)
  {
    if (id == "image_in") {
      return image_sub_3_;
    } else {
      RCLCPP_ERROR(get_logger(), "Subscriber [%s] not found", id.c_str());
      return nullptr;
    }
  }
};


class ConsumerBase2 : public rclcpp::Node
{
public:
  ConsumerBase2();

protected:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_0_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_sub_1_;

  virtual void image_sub_0_callback(sensor_msgs::msg::Image::SharedPtr msg) = 0;
  virtual void string_sub_1_callback(std_msgs::msg::String::SharedPtr msg) = 0;

  template<typename T>
  typename rclcpp::Publisher<T>::SharedPtr
  get_publisher(const std::string & id)
  {
    auto ret_pub = std::dynamic_pointer_cast<typename rclcpp::Publisher<T>>(
      get_publisher_base(id));
    return ret_pub;
  }

  template<typename T>
  typename rclcpp::Subscription<T>::SharedPtr
  get_subscription(const std::string & id)
  {
    auto ret_sub = std::dynamic_pointer_cast<typename rclcpp::Subscription<T>>(
      get_subscription_base(id));
    return ret_sub;
  }

  typename rclcpp::PublisherBase::SharedPtr
  get_publisher_base(const std::string & id)
  {
    RCLCPP_ERROR(get_logger(), "Not publishers found");
    return nullptr;
  }

  typename rclcpp::SubscriptionBase::SharedPtr
  get_subscription_base(const std::string & id)
  {
    if (id == "image_in") {
      return image_sub_0_;
    } else if (id == "description_in") {
      return string_sub_1_;
    } else {
      RCLCPP_ERROR(get_logger(), "Subscriber [%s] not found", id.c_str());
      return nullptr;
    }
  }
};

}  // namespace rossdl_test

#endif  // ROSSDL_TEST__NODES2_HPP_

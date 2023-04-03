// generated from rossdl_cmake/resource/nodes.hpp.em
// generated code does not contain a copyright notice

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

}  // namespace rossdl_test

#endif  // ROSSDL_TEST__NODES2_HPP_

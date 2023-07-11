// generated from rossdl_cmake/resource/nodes.hpp.em
// generated code does not contain a copyright notice

@{
from rossdl_cmake import get_header_guard
from rossdl_cmake import get_message_headers
from rossdl_cmake import get_node_names
from rossdl_cmake import get_class_names_from_node
from rossdl_cmake import get_publishers_name_type_from_node
from rossdl_cmake import get_subscriptions_name_type_from_node


arfifacts = locals()['artifacts']
package_name = locals()['package']

header_guard = get_header_guard(package_name)
node_names = get_node_names(package_name, arfifacts)
node_class_names = []
for node_name in node_names:
    node_class_names.append((node_name, get_class_names_from_node(node_name)))

}@
#ifndef @(header_guard)
#define @(header_guard)

#include <string>
#include <optional>
#include <typeinfo>

@{
msg_headers = get_message_headers(package_name, arfifacts)
}@
@[for header_file in msg_headers]@
#include "@(header_file)"
@[end for]@

#include "rclcpp/rclcpp.hpp"

namespace @(package_name)
{

@[for node_name in node_class_names]@
class @(node_name[1])Base : public rclcpp::Node
{
public:
  explicit @(node_name[1])Base(const rclcpp::NodeOptions & options);

protected:
@{
publishers_info = get_publishers_name_type_from_node(package_name, arfifacts, node_name[0])
}@
@[    for publisher_info in publishers_info]@
  rclcpp::Publisher<@(publisher_info[1])>::SharedPtr @(publisher_info[0])_;
@[    end for]@
@{
subscribers_info = get_subscriptions_name_type_from_node(package_name, arfifacts, node_name[0])
}@
@[    for subscriber_info in subscribers_info]@
  rclcpp::Subscription<@(subscriber_info[1])>::SharedPtr @(subscriber_info[0])_;
@[    end for]@

@[    for subscriber_info in subscribers_info]@
  virtual void @(subscriber_info[0])_callback(@(subscriber_info[1])::SharedPtr msg) = 0;
@[    end for]@

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
@[    for publisher_info in publishers_info]@
    if (id == "@(publisher_info[0])") {
        return @(publisher_info[0])_;
    } 
@[    end for]@
    RCLCPP_ERROR(get_logger(), "Publisher [%s] not found", id.c_str());
    return nullptr;
  }

  typename rclcpp::SubscriptionBase::SharedPtr
  get_subscription_base(const std::string & id)
  {
@[    for subscriber_info in subscribers_info]@
    if (id == "@(subscriber_info[0])") {
        return @(subscriber_info[0])_;
    } 
@[    end for]@
    RCLCPP_ERROR(get_logger(), "Subscriber [%s] not found", id.c_str());
    return nullptr;
  }
};

@[end for]@

}  // namespace @(package_name)

#endif  // @(header_guard)
   
// generated from rossdl_cmake/resource/nodes.hpp.em
// generated code does not contain a copyright notice

#include <optional>

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

#include "rossdl_test/Nodes2.hpp"

#include "rclcpp/rclcpp.hpp"

namespace rossdl_test
{

using std::placeholders::_1;

ImageFilterBase2::ImageFilterBase2()
: Node("image_filter")
{
  image_pub_0_ = create_publisher<sensor_msgs::msg::Image>(
    std::string(get_fully_qualified_name()) + "/image_out",
    rclcpp::SensorDataQoS());
  string_pub_1_ = create_publisher<std_msgs::msg::String>(
    std::string(get_fully_qualified_name()) + "/description_out",
    100);
  image_sub_3_ = create_subscription<sensor_msgs::msg::Image>(
    std::string(get_fully_qualified_name()) + "/image_in",
    rclcpp::SensorDataQoS().reliable(),
    std::bind(&ImageFilterBase2::image_sub_3_callback, this, _1));
}

}  // namespace rossdl_test

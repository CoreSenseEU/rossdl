// generated from rossdl_cmake/resource/nodes.hpp.em
// generated code does not contain a copyright notice

@{
from rossdl_cmake import get_header_guard
from rossdl_cmake import get_message_headers
from rossdl_cmake import get_node_names
from rossdl_cmake import get_class_names_from_node
from rossdl_cmake import get_publishers_name_type_from_node
from rossdl_cmake import get_subscriptions_name_type_from_node
from rossdl_cmake import get_qos_from_node_topic
from rossdl_cmake import get_parameters_info

arfifacts = locals()['artifacts']
package_name = locals()['package']

node_names = get_node_names(package_name, arfifacts)

node_class_names = []
for node_name in node_names:
    node_class_names.append((node_name, get_class_names_from_node(node_name)))

}@

@{
msg_headers = get_message_headers(package_name, arfifacts)
}@
@[for header_file in msg_headers]@
#include "@(header_file)"
@[end for]@

#include "@(package_name)/Nodes.hpp"

#include "rclcpp/rclcpp.hpp"

namespace @(package_name)
{

using std::placeholders::_1;

@[for node_name in node_class_names]@
@{
publishers_info = get_publishers_name_type_from_node(package_name, arfifacts, node_name[0])
subscribers_info = get_subscriptions_name_type_from_node(package_name, arfifacts, node_name[0])
}@
@(node_name[1])Base::@(node_name[1])Base(const rclcpp::NodeOptions & options)
: Node("@(node_name[0])", options)
{
@[    for publisher_info in publishers_info]@
  @(publisher_info[0])_ = create_publisher<@(publisher_info[1])>(
    std::string(get_fully_qualified_name()) + "/@(publisher_info[0])",
@{
qos = get_qos_from_node_topic(package_name, arfifacts, node_name[0], publisher_info[0])
}@     @(qos));
@[    end for]@
@[    for subscriber_info in subscribers_info]@
  @(subscriber_info[0])_ = create_subscription<@(subscriber_info[1])>(
    std::string(get_fully_qualified_name()) + "/@(subscriber_info[0])",
@{qos = get_qos_from_node_topic(package_name, arfifacts, node_name[0], subscriber_info[0])}@
    @(qos),
    std::bind(&@(node_name[1])Base::@(subscriber_info[0])_callback, this, _1));
@[    end for]@
@{parameters = get_parameters_info(package_name, arfifacts, node_name[0])}@
@[    for parameter in parameters]@
@[        if parameter[1] == "std::string"]
  declare_parameter("@(parameter[0])", @(parameter[1])("@(parameter[2])"));
@[        else]
  declare_parameter("@(parameter[0])", @(parameter[1])(@(parameter[2])));
@[        end if]@
@[    end for]@
};

@[end for]@
}  // namespace @(package_name)

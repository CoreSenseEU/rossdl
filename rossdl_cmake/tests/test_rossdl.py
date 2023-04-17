# Copyright 2022 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#


import os
import unittest

from ament_index_python.packages import get_package_share_directory

import pytest

import rossdl_cmake


@pytest.mark.rostest
class TestROSSDL(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.filename = os.path.join(get_package_share_directory(
            'rossdl_cmake'), 'description.sdl')

    def test_read_description(self):
        yaml_data = rossdl_cmake.read_description(self.filename)
        self.assertNotEqual(yaml_data, {})

    def test_get_package_name(self):
        yaml_data = rossdl_cmake.read_description(self.filename)
        self.assertEqual(rossdl_cmake.get_package_name(yaml_data), 'rossdl_test')

    def test_get_node_names(self):
        yaml_data = rossdl_cmake.read_description(self.filename)
        self.assertEqual(rossdl_cmake.get_node_names(yaml_data),
                         ['image_filter', 'consumer'])

    def test_get_publishers_name_type_from_node(self):
        yaml_data = rossdl_cmake.read_description(self.filename)
        self.assertEqual(
            rossdl_cmake.get_publishers_name_type_from_node(yaml_data, 'image_filter'),
            [('image_out', 'sensor_msgs::msg::Image'),
             ('description_out', 'std_msgs::msg::String')
             ])
        self.assertEqual(
            rossdl_cmake.get_publishers_name_type_from_node(yaml_data, 'consumer'),
            [('image_out', 'sensor_msgs::msg::Image')])

    def test_get_qos_from_node_topic(self):
        yaml_data = rossdl_cmake.read_description(self.filename)

        self.assertEqual(
            rossdl_cmake.get_qos_from_node_topic(yaml_data, 'image_filter', 'image_out'),
            'rclcpp::SensorDataQoS().reliable()')
        self.assertEqual(
            rossdl_cmake.get_qos_from_node_topic(yaml_data, 'image_filter', 'description_out'),
            'rclcpp::QoS(100)')
        self.assertEqual(
            rossdl_cmake.get_qos_from_node_topic(yaml_data, 'image_filter', 'image_in'),
            'rclcpp::SensorDataQoS()')
        self.assertEqual(
            rossdl_cmake.get_qos_from_node_topic(yaml_data, 'image_filter', 'laser_in'),
            'rclcpp::SensorDataQoS().reliable()')
        self.assertEqual(
            rossdl_cmake.get_qos_from_node_topic(yaml_data, 'consumer', 'image_in'),
            'rclcpp::SensorDataQoS()')
        self.assertEqual(
            rossdl_cmake.get_qos_from_node_topic(yaml_data, 'consumer', 'description_in'),
            '100')
        self.assertEqual(
            rossdl_cmake.get_qos_from_node_topic(yaml_data, 'consumer', 'image_out'),
            'rclcpp::SensorDataQoS()')

    def test_get_subscriptions_name_type_from_node(self):
        yaml_data = rossdl_cmake.read_description(self.filename)
        self.assertEqual(
            rossdl_cmake.get_subscriptions_name_type_from_node(yaml_data, 'image_filter'),
            [('image_in', 'sensor_msgs::msg::Image'),
             ('laser_in', 'sensor_msgs::msg::LaserScan')
             ])
        self.assertEqual(
            rossdl_cmake.get_subscriptions_name_type_from_node(yaml_data, 'consumer'),
            [('image_in', 'sensor_msgs::msg::Image'),
             ('description_in', 'std_msgs::msg::String')
             ])

    def test_get_class_names_from_node(self):
        self.assertEqual(
            rossdl_cmake.get_class_names_from_node('image_filter'),
            'ImageFilter')
        self.assertEqual(
            rossdl_cmake.get_class_names_from_node('consumer'),
            'Consumer')

    def test_get_header_guard(self):
        yaml_data = rossdl_cmake.read_description(self.filename)
        self.assertEqual(
            rossdl_cmake.get_header_guard(yaml_data),
            'ROSSDL_TEST__NODES_HPP_')

    def test_get_message_header_from_type(self):
        self.assertEqual(
            rossdl_cmake.get_message_header_from_type('std_msgs/msg/String'),
            'std_msgs/msg/string.hpp')
        self.assertEqual(
            rossdl_cmake.get_message_header_from_type('sensor_msgs/msg/Image'),
            'sensor_msgs/msg/image.hpp')
        self.assertEqual(
            rossdl_cmake.get_message_header_from_type('sensor_msgs/msg/LaserScan'),
            'sensor_msgs/msg/laser_scan.hpp')
        self.assertEqual(
            rossdl_cmake.get_message_header_from_type('sensor_msgs/msg/PointCloud2'),
            'sensor_msgs/msg/point_cloud2.hpp')

    def test_to_cpp_type(self):
        self.assertEqual(
            rossdl_cmake.to_cpp_type('string'),
            'std::string')
        self.assertEqual(
            rossdl_cmake.to_cpp_type('float'),
            'float')

    def test_get_parameters_info(self):
        yaml_data = rossdl_cmake.read_description(self.filename)
        self.assertEqual(
            rossdl_cmake.get_parameters_info(yaml_data, 'image_filter'),
            [('description_label', 'std::string', 'default image')])
        self.assertEqual(
            rossdl_cmake.get_parameters_info(yaml_data, 'consumer'),
            [])

    def test_generate_cpp(self):
        good_cpp_content = ''
        good_hpp_content = ''
        test_cpp_content = ''
        test_hpp_content = ''

        good_hpp_filename = os.path.join(get_package_share_directory(
            'rossdl_cmake'), 'Nodes.hpp.test')
        good_cpp_filename = os.path.join(get_package_share_directory(
            'rossdl_cmake'), 'Nodes.cpp.test')
        test_hpp_filename = '/tmp/Nodes.hpp'
        test_cpp_filename = '/tmp/Nodes.cpp'

        rossdl_cmake.generate_cpp(self.filename, test_hpp_filename, test_cpp_filename)

        with open(good_hpp_filename) as f:
            good_hpp_content = f.read()
        with open(good_cpp_filename) as f:
            good_cpp_content = f.read()
        with open(test_hpp_filename) as f:
            test_hpp_content = f.read()
        with open(test_cpp_filename) as f:
            test_cpp_content = f.read()

        self.assertEqual(len(good_hpp_content), len(test_hpp_content))
        self.assertEqual(len(good_cpp_content), len(test_cpp_content))

    def test_get_system_remappings(self):
        yaml_data = rossdl_cmake.read_description(self.filename)
        remappings = rossdl_cmake.get_system_remappings(yaml_data['rossdl_test'], 'system_1')

        self.assertEqual(
            remappings,
            {
                'consumer': [
                    ('/consumer/image_out', '/other_node/image_in'),
                ],
                'image_filter': [
                    ('/image_filter/image_out', '/consumer/image_in'),
                    ('/image_filter/description_out', '/consumer/description_in'),
                    ('/image_filter/image_in', '/camera/rgb/image_raw'),
                ]
            })

    def test_get_system_parameters(self):
        yaml_data = rossdl_cmake.read_description(self.filename)
        data_and_system = {}
        data_and_system['data'] = yaml_data
        data_and_system['system'] = 'rossdl_test'

        parameters = rossdl_cmake.get_system_parameters(yaml_data['rossdl_test'], 'system_1')

        self.assertEqual(
            parameters,
            ({
                'image_filter': [
                    ('description_label', 'image raw'),
                    ('use_sim_time', False)],
                'consumer': [
                    ('use_sim_time', False)]
            }))

    def test_get_system_to_generate(self):
        yaml_data = rossdl_cmake.read_description(self.filename)
        data_and_system = {}
        data_and_system['data'] = yaml_data
        data_and_system['system'] = 'rossdl_test'

        self.assertEqual(
            rossdl_cmake.get_system_to_generate(data_and_system),
            'rossdl_test')

    def test_get_system_nodes(self):
        yaml_data = rossdl_cmake.read_description(self.filename)
        data_and_system = {}
        data_and_system['data'] = yaml_data
        data_and_system['system'] = 'rossdl_test'

        self.assertEqual(
            rossdl_cmake.get_system_nodes(data_and_system, 'system_1'),
            [
                ('image_filter', 'rossdl_test::ImageFilter'),
                ('consumer', 'rossdl_test::Consumer')
            ])

#     def test_generate_system(self):
#         good_launch_content = ''
#         test_launch_content = ''
#
#         good_launch_filename = os.path.join(get_package_share_directory(
#             'rossdl_cmake'), 'system_1.launch.py.test')
#         test_launch_filename = '/tmp/system_1.launch.py'
#
#         rossdl_cmake.generate_system(self.filename, test_launch_filename, 'system_1')
#
#         with open(good_launch_filename) as f:
#             good_launch_content = f.read()
#         with open(test_launch_filename) as f:
#             test_launch_content = f.read()
#
#         self.assertEqual(len(good_launch_content), len(test_launch_content))


if __name__ == '__main__':
    unittest.main()

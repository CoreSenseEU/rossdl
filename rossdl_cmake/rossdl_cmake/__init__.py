# Copyright 2015 Open Source Robotics Foundation, Inc.
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

from io import StringIO
import os
import sys

from ament_index_python.packages import get_package_share_directory

import em
import yaml
from yaml.loader import SafeLoader

def read_description(file_in):
    with open(file_in) as f:
        data = yaml.load(f, Loader=SafeLoader)
        return data

def get_package_name(data):
    return list(data.keys())[0]

def get_node_names(data):
    pkg_name = list(data.keys())[0]
    pkg_artifacts = data[pkg_name]['artifacts']
    return list(pkg_artifacts.keys())

def get_publishers_name_type_from_node(data, node):
    pkg_name = list(data.keys())[0]
    pkg_artifacts = data[pkg_name]['artifacts']

    ret = []    
    if node in pkg_artifacts:
        node_content = pkg_artifacts[node]
        if 'publishers' in node_content:
            publishers = list(node_content['publishers'].keys())
            for publisher in publishers:
                publisher_info = node_content['publishers'][publisher]
                type = publisher_info['type'].replace('/', '::')
                ret.append((publisher, type))
    return ret

def get_qos_from_data(data):
    ret = ""
    if 'qos_history_depth' in data:
        ret = 'rclcpp::QoS(' + str(data['qos_history_depth']) + ")"
    if 'qos_profile' in data:
        if data['qos_profile'] == 'sensor_qos':
            ret = 'rclcpp::SensorDataQoS()'
    if 'qos_reliability' in data:
        if data['qos_reliability'] == 'reliable':
            ret = ret + '.reliable()'
        elif data['qos_reliability'] == 'best effort':
            ret = ret + '.BestEffort()'
    ## Here we need to comple all the options
    return ret

def get_qos_from_node_topic(data, node, topic):
    pkg_name = list(data.keys())[0]
    pkg_artifacts = data[pkg_name]['artifacts']
    pkg_nodes = list(pkg_artifacts.keys())

    if node in pkg_artifacts:
        node_content = pkg_artifacts[node]
        if 'publishers' in node_content:
            publishers = list(node_content['publishers'].keys())
            if topic in publishers:
                publisher_info = node_content['publishers'][topic]
                if 'qos' in publisher_info:
                    return get_qos_from_data(publisher_info['qos'])
                else:
                    return '100'
        if 'subscribers' in node_content:
            subscribers = list(node_content['subscribers'].keys())
            if topic in subscribers:
                subscriber_info = node_content['subscribers'][topic]
                if 'qos' in subscriber_info:
                    return get_qos_from_data(subscriber_info['qos'])
                else:
                    return '100'

def get_subscriptions_name_type_from_node(data, node):
    pkg_name = list(data.keys())[0]
    pkg_artifacts = data[pkg_name]['artifacts']

    ret = []    
    if node in pkg_artifacts:
        node_content = pkg_artifacts[node]
        if 'subscribers' in node_content:
            subscribers = list(node_content['subscribers'].keys())
            for subscriber in subscribers:
                subscriber_info = node_content['subscribers'][subscriber]
                type = subscriber_info['type'].replace('/', '::')
                ret.append((subscriber, type))
    return ret

def get_class_names_from_node(node):
    node_list = list(node)
    output = []

    for i in range(len(node_list)):
        if i == 0 or node_list[i-1] == '_':
            output.append(node_list[i].upper())
        elif node_list[i] != '_':
            output.append(node_list[i])
    return "".join(output) 

def get_header_guard(data):
    pkg_name = list(data.keys())[0]
    return pkg_name.upper() + "__NODES_HPP_"

def get_message_header_from_type(msg_type):
    msg_type_list = list(msg_type)
    output = []
    for i in range(len(msg_type_list)):
        if msg_type_list[i].isupper():
            if msg_type_list[i - 1] != "/":
                output.append("_")          
            output.append(msg_type[i].lower())
        else:
            output.append(msg_type_list[i])
    return "".join(output) + ".hpp"

def to_cpp_type(type):
    if type == 'string':
        return 'std::string'
    elif type == 'float':
        return 'float'
    ## Here we need to comple all the options

def get_parameters_info(data, node):
    pkg_name = list(data.keys())[0]
    pkg_artifacts = data[pkg_name]['artifacts']
    pkg_nodes = list(pkg_artifacts.keys())  

    ret = []
    if node in pkg_artifacts:
        node_content = pkg_artifacts[node]
        if 'parameters' in node_content:
            parameters = list(node_content['parameters'].keys())
            for parameter in parameters:
                parameter_info = node_content['parameters'][parameter]
                ret.append((parameter,
                            to_cpp_type(parameter_info['type']),
                            parameter_info['default']))
    return ret

def get_message_headers(data):
    pkg_name = list(data.keys())[0]
    pkg_artifacts = data[pkg_name]['artifacts']
    pkg_nodes = list(pkg_artifacts.keys())

    msg_types = set()
    for pkg_node in pkg_nodes:
        node_content = pkg_artifacts[pkg_node]
        if 'publishers' in node_content:
            publishers = list(node_content['publishers'].keys())
            for publisher in publishers:
                publisher_info = node_content['publishers'][publisher]
                msg_types.add(publisher_info['type'])
        if 'subscribers' in node_content:
            subscribers = list(node_content['subscribers'].keys())
            for subscriber in subscribers:
                subscriber_info = node_content['subscribers'][subscriber]
                msg_types.add(subscriber_info['type'])

    headers_ret = []
    for msg_type in msg_types:
        headers_ret.append(get_message_header_from_type(msg_type))
    return headers_ret

def generate_cpp(file_in, header_out, source_out):
    resources_dir = os.path.join(get_package_share_directory('rossdl_cmake'), 'resources')
    header_in = os.path.join(resources_dir, 'nodes.hpp.em')
    source_in = os.path.join(resources_dir, 'nodes.cpp.em')

    data = read_description(file_in)

    generate_file(header_in, header_out, data)
    generate_file(source_in, source_out, data)


def generate_file(file_in, file_out, data):
    file_output = StringIO()
    content = ''
    try:
        _interpreter = em.Interpreter(
            output=file_output,
            options={
                em.BUFFERED_OPT: True,
                em.RAW_OPT: True,
            })

        with open(file_in, 'r') as h:
            content = h.read()
        _interpreter.invoke(
            'beforeFile', name=file_in, file=h, locals=data)
        _interpreter.string(content, file_in, locals=data)
        _interpreter.invoke('afterFile')
        content = file_output.getvalue()
    except Exception as e:  # noqa: F841
        print(
            f"{e.__class__.__name__} processing template '{file_in}'",
            file=sys.stderr)
        raise
    finally:
        _interpreter.shutdown()
        _interpreter = None

    if os.path.exists(file_out):
        timestamp_in = os.path.getmtime(file_in)
        timestamp_out = None
        try:
            timestamp_out = os.path.getmtime(file_out)
        except FileNotFoundError:
            pass
        if False and timestamp_out is not None and timestamp_out < timestamp_in:
            with open(file_out, 'r', encoding='utf-8') as h:
                if h.read() == content:
                    return
    else:
        try:
            os.makedirs(os.path.dirname(file_out))
        except FileExistsError:
            pass

    with open(file_out, 'w', encoding='utf-8') as h:
        h.write(content)

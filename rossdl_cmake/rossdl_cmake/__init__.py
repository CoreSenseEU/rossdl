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
from ament_index_python.resources import get_resource, get_search_paths
import em
import yaml
from yaml.loader import SafeLoader


def read_description(file_in):
    with open(file_in) as f:
        data = yaml.load(f, Loader=SafeLoader)
        return data


def get_node_names(package, artifacts):
    pkg_artifacts = artifacts[package]['artifacts']
    return list(pkg_artifacts.keys())


def get_publishers_name_type_from_node(package, arfifacts, node):
    pkg_artifacts = arfifacts[package]['artifacts']

    ret = []
    if node in pkg_artifacts:
        node_content = pkg_artifacts[node]
        if 'publishers' in node_content:
            publishers = list(node_content['publishers'].keys())
            for publisher in publishers:
                publisher_info = node_content['publishers'][publisher]
                pub_type = publisher_info['type'].replace('/', '::')
                ret.append((publisher, pub_type))
    return ret


def get_qos_from_data(data):
    ret = ''
    if 'qos_history_depth' in data:
        ret = 'rclcpp::QoS(' + str(data['qos_history_depth']) + ')'
    if 'qos_profile' in data:
        if data['qos_profile'] == 'sensor_qos':
            ret = 'rclcpp::SensorDataQoS()'
    if 'qos_reliability' in data:
        if data['qos_reliability'] == 'reliable':
            ret = ret + '.reliable()'
        elif data['qos_reliability'] == 'best effort':
            ret = ret + '.BestEffort()'
    # Here we need to comple all the options
    return ret


def get_qos_from_node_topic(package, arfifacts, node, topic):
    pkg_artifacts = arfifacts[package]['artifacts']

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


def get_subscriptions_name_type_from_node(package, arfifacts, node):
    pkg_artifacts = arfifacts[package]['artifacts']

    ret = []
    if node in pkg_artifacts:
        node_content = pkg_artifacts[node]
        if 'subscribers' in node_content:
            subscribers = list(node_content['subscribers'].keys())
            for subscriber in subscribers:
                subscriber_info = node_content['subscribers'][subscriber]
                sub_type = subscriber_info['type'].replace('/', '::')
                ret.append((subscriber, sub_type))
    return ret


def get_class_names_from_node(node):
    node_list = list(node)
    output = []

    for i in range(len(node_list)):
        if i == 0 or node_list[i-1] == '_':
            output.append(node_list[i].upper())
        elif node_list[i] != '_':
            output.append(node_list[i])
    return ''.join(output)


def get_header_guard(pkg_name):
    return pkg_name.upper() + '__NODES_HPP_'


def get_message_header_from_type(msg_type):
    msg_type_list = list(msg_type)
    output = []
    for i in range(len(msg_type_list)):
        if msg_type_list[i].isupper():
            if msg_type_list[i - 1] != '/':
                output.append('_')
            output.append(msg_type[i].lower())
        else:
            output.append(msg_type_list[i])
    return ''.join(output) + '.hpp'


def to_cpp_type(ptype):
    if ptype == 'string':
        return 'std::string'
    elif ptype == 'float':
        return 'float'
    # Here we need to comple all the options


def get_parameters_info(package, arfifacts, node):
    pkg_artifacts = arfifacts[package]['artifacts']

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


def get_message_headers(package, arfifacts):
    pkg_artifacts = arfifacts[package]['artifacts']
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


def generate_cpp(package, file_in, header_out, source_out):
    resources_dir = os.path.join(get_package_share_directory('rossdl_cmake'), 'resources')
    header_in = os.path.join(resources_dir, 'nodes.hpp.em')
    source_in = os.path.join(resources_dir, 'nodes.cpp.em')

    generate_file(package, file_in, header_in, header_out)
    generate_file(package, file_in, source_in, source_out)


def generate_file(package, artifacts_file, file_in, file_out):
    data = {}
    data['artifacts'] = read_description(artifacts_file)
    data['package'] = package

    # print(data)

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
        if timestamp_out is not None and timestamp_out < timestamp_in:
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


def get_system_remappings(system_info, arfifacts):
    connections = system_info['connections']
    
    remappings = {}
    for connection in connections:
        origin = connection[0].split('/')[1]
        destiny = connection[1].split('/')[1]

        if origin in list(arfifacts.keys()):
            if origin not in remappings.keys():
                remappings[origin] = []
            remappings[origin].append((connection[0], connection[1]))
        elif destiny in list(arfifacts.keys()):
            if destiny not in remappings.keys():
                remappings[destiny] = []
            remappings[destiny].append((connection[1], connection[0]))
    return remappings


def get_system_parameters(system_info, arfifacts):
    parameters = system_info['parameters']

    parameters_ret = {}
    for parameter in parameters:
        parameter_name = parameter[0].split('/')[2]
        node_name = parameter[0].split('/')[1]
        value = parameter[1]

        if node_name == '*':
            for node in list(arfifacts.keys()):
                if node not in parameters_ret.keys():
                    parameters_ret[node] = []
                parameters_ret[node].append((parameter_name, value))
        else:
            if node_name not in parameters_ret.keys():
                parameters_ret[node_name] = []
            parameters_ret[node_name].append((parameter_name, value))

    return parameters_ret


def get_system_nodes(system_info):
    node_names = system_info['nodes']
    ret = []
    for node in node_names:
        pkg = node.split('::')[0]
        class_name = get_class_names_from_node(node.split('::')[1])
        ret.append((node.split('::')[1], pkg + '::' + class_name))
    return ret


def generate_launch(package, file_in, launch_in, file_out, system, systems_data, artifacts_data):
    data_and_system = {}
    data_and_system['system'] = system
    data_and_system['package'] = package
    data_and_system['systems_data'] = systems_data
    data_and_system['artifacts'] = artifacts_data

    print(artifacts_data)

    file_output = StringIO()
    content = ''
    try:
        _interpreter = em.Interpreter(
            output=file_output,
            options={
                em.BUFFERED_OPT: True,
                em.RAW_OPT: True,
            })

        with open(launch_in, 'r') as h:
            content = h.read()
        _interpreter.invoke(
            'beforeFile', name=file_in, file=h, locals=data_and_system)
        _interpreter.string(content, launch_in, locals=data_and_system)
        _interpreter.invoke('afterFile')
        content = file_output.getvalue()
    except Exception as e:  # noqa: F841
        print(
            f"{e.__class__.__name__} processing template '{launch_in}'",
            file=sys.stderr)
        raise
    finally:
        _interpreter.shutdown()
        _interpreter = None

    if os.path.exists(file_out):
        timestamp_in = os.path.getmtime(launch_in)
        timestamp_out = None
        try:
            timestamp_out = os.path.getmtime(file_out)
        except FileNotFoundError:
            pass
        if timestamp_out is not None and timestamp_out < timestamp_in:
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

def get_artifacts_data(artifacts):
    artifact_contents = [get_resource('rossdl_artifact_descriptions', a) for a in artifacts]
    artifact_content = ''
    for c in artifact_contents:
        artifact_content = artifact_content + c[0]

    artifacts_data = yaml.safe_load_all(artifact_content)   

    return artifacts_data

def get_data_resource(ids, resource):
    try:
        contents = [get_resource(resource, a) for a in ids]
    
        content = ''
        for c in contents:
            content = content + c[0]

        data = yaml.load(content, Loader=SafeLoader)

        return data
    except LookupError:
        print("Resources are not ready. Maybe you should source your workspace and build again")
        return {}

def generate_system(package, file_in, file_out, system, artifacts, local_artifacts, systems, local_systems):
    resources_dir = os.path.join(get_package_share_directory('rossdl_cmake'), 'resources')
    launch_in = os.path.join(resources_dir, 'launcher.py.em')

    print('===========================================')
    print('Genarating System {}'.format(system))
    print('artifacts: {}'.format(artifacts))
    print('local_artifacts: {}'.format(local_artifacts))
    print('systems: {}'.format(systems))
    print('local_systems: {}'.format(local_systems))
    print('===========================================')

    artifacts = [x for x in artifacts if x != 'None']
    local_artifacts = [x for x in local_artifacts if x != 'None']
    systems = [x for x in systems if x != 'None']
    local_systems = [x for x in local_systems if x != 'None']

    artifacts_data = {}
    systems_data = {}
    
    if len(artifacts) > 0:
        artifacts_data = get_data_resource(artifacts, 'rossdl_artifact_descriptions')
    
    print('Reading local artifacts')
    try:
        if len(local_artifacts) > 0:
            for local_artficact in local_artifacts:
                print('Reading ' + local_artficact)
                with open(local_artficact, 'r') as f:
                    local_artifact_data = yaml.load(f.read(), Loader=SafeLoader)
                    print(local_artifact_data),
                    print('------>'),
                    artifacts_data.update(local_artifact_data)
                    print(artifacts_data)
    except LookupError:
        print("Error reading local_artifacts")

    if len(systems) > 0:
        systems_data = get_data_resource(systems, 'rossdl_system_descriptions')

    print('Reading local systems')
    try:
        if len(local_systems) > 0:
            for local_systems in local_systems:
                print('Reading ' + local_systems)
                with open(local_systems, 'r') as f:
                    local_systems_data = yaml.load(f.read(), Loader=SafeLoader)
                    print(local_systems_data),
                    print('------>'),
                    systems_data.update(local_systems_data)
                    print(systems_data)
    except LookupError:
        print("Error reading local_artifacts")

    print("Artifacts =============================")
    print(artifacts)
    print("Systems =============================")
    print(systems)
    print("Artifacts Data =============================")
    print(artifacts_data)
    print("Systems =============================")
    print(systems_data)

    ## ACABO DE METER LO DE FILE_IN PORQUE PARECE QUE NO USA EL FICHERO DE ENTRADA
    generate_launch(package, file_in, launch_in, file_out, system, systems_data, artifacts_data)
 
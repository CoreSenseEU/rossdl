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


def generate_cpp(file_in, header_out, source_out, data):
    resources_dir = os.path.join(get_package_share_directory('rossdl_cmake'), 'resources')
    header_in = os.path.join(resources_dir, 'nodes.hpp.em')
    source_in = os.path.join(resources_dir, 'nodes.cpp.em')

    # print("I should generate files from {}".format(file_in))
    # print("\tHeader IN {}".format(header_in))
    # print("\tSource IN {}".format(source_in))
    # print("\tHeader OUT {}".format(header_out))
    # print("\tSource OUT {}".format(source_out))

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
        # print(content)
    except Exception as e:  # noqa: F841
        print(
            f"{e.__class__.__name__} processing template '{file_in}'",
            file=sys.stderr)
        raise
    finally:
        _interpreter.shutdown()
        _interpreter = None

    if os.path.exists(file_out):
        # print('path {} existe'.format(file_out))
        timestamp_in = os.path.getmtime(file_in)
        timestamp_out = None
        try:
            timestamp_out = os.path.getmtime(file_out)
        except FileNotFoundError:
            pass
        if timestamp_out is not None or timestamp_out > timestamp_in:
            with open(file_out, 'r', encoding='utf-8') as h:
                if h.read() == content:
                    return
    else:
        # print('archivo NO existe')
        try:
            os.makedirs(os.path.dirname(file_out))
        except FileExistsError:
            # print('FileExistsError error')
            pass

    with open(file_out, 'w', encoding='utf-8') as h:
        h.write(content)

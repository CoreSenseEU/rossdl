# ROSSDL (ROS System Definition Language)

Authors:
* Francisco Martín Rico - fmrico@gmail.com
* Nadia Hammoudeh García - nadia.hammoudeh.garcia@ipa.fraunhofer.de 

[![rolling](https://github.com/fmrico/rossdl/actions/workflows/rolling.yaml/badge.svg)](https://github.com/fmrico/rossdl/actions/workflows/rolling.yaml)

*Brave ROS developers like to code*. ***But they like even more that their applications work***.

What is clear is that they do not like to plan their applications by applying tedious Software Engineering and Model-driven Engineering techniques, which require drawing boxes and connections at different levels and from different perspectives. But sometimes applications become large and unwieldy, and these software engineering techniques are required to validate, verify, and certify that the applications are correct, like it or not.

![ROSSDL_idea](https://github.com/user-attachments/assets/24bdacfe-bac3-4b06-a062-c2021082c522)

ROSSDL provides a tool that is in the middle of these two perspectives and is useful for both worlds: Programmers can continue writing lines of code, and software engineers are provided with a model-based file format where they can dump the output of their tools and take inputs for verification tasks.

## Quick trip through ROSSDL

Let's imagine that we need to develop an application in ROS 2 that looks like the following diagram:

![ROSSDL_simple_0](https://github.com/user-attachments/assets/e7e7356a-2d6d-438c-82d8-f248c11d71ef)

This is an application that has two nodes. The first, the Image Filter, subscribes to images from a camera and a laser range sensor to filter and merge these two inputs. The output of this node is received by a Consumer node that does some processing and publishes its result.

This is a simple application, but it already presents some challenges that waste a lot of a programmer's time. Connecting inputs from one node to outputs from another may be trivial, but it is a common point of failure due to topic names and qualities of service. Also, these nodes probably have some parameters that need to be kept track of.

In ROSSDL we are going to differentiate two different moments, which we often mix up: **Development** and **deployment**.

* By ***development***, we mean the development of the nodes that have to do with the application individually. The following diagram represents the simplified development planning of the application nodes:

![ROSSDL_simple_1](https://github.com/user-attachments/assets/8bb716d7-347b-4449-9585-f2af746f839a)

* By ***deployment***, we mean how we configure our application components to be executed. In ROS 2, we typically do this by using launchers, which define parameters, connect nodes, and define all the details. Let's look at a diagram that shows deployment:

![ROSSDL_simple_2](https://github.com/user-attachments/assets/c66a111a-6384-4606-b63b-9d760d272a02)

### Development with ROSSDL

To use ROSSDL, you simply create `.ros2` and `.rossystem` files in your package. 

![image](https://github.com/user-attachments/assets/7bae0b70-4866-41db-9d74-e83ee697c957)

These Yaml files define the nodes to develop with publishers, subscribers, and parameters (services and actions are not supported yet). In the development phase, we are only interested in the ".ros2" file, which looks like this:

```
---
rossdl_simple_test:
  fromGitRepo: "https://github.com/jane-doe/project_example.git:branch"
  artifacts:
    image_filter:
      node: "image_filter"
      publishers:
        image_out:
          type: "sensor_msgs/msg/Image"
        description_out:
          type: "std_msgs/msg/String"
      subscribers:
        image_in:
          type: "sensor_msgs/msg/Image"
        laser_in:
          type: "sensor_msgs/msg/LaserScan"
      parameters:
        description_label:
          type: string
          default: "default image"
    consumer:
      node: "consumer"
      subscribers:
        image_in:
          type: "sensor_msgs/msg/Image"
        description_in:
          type: "std_msgs/msg/String"
      publishers:
        image_out:
          type: "sensor_msgs/msg/Image"
```

When the workspace is compiled using `colcon build`, this file is read and the skeleton of the nodes is generated in the build directory. This approach is crucial as it prevents the source directory from being cluttered with generated code. The generated code consists of base classes for the nodes, which abstract the complexity of managing publishers, subscribers, and parameters. Consequently, the programmer only needs to define the application logic by inheriting from these base classes and adhering to some basic conventions and mechanisms, for example:

- If your node is called `consumer`, the base class will be `ConsumerBase`.
- If you have a subscriber called `image_in`, its callback will be `image_in_callback`.
- All topic names include as a prefix the node name.
  
```
Consumer::Consumer(const rclcpp::NodeOptions & options)
: ConsumerBase(options)
{
}

void
Consumer::image_in_callback(sensor_msgs::msg::Image::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Image message received");
  auto pub = get_publisher<sensor_msgs::msg::Image>("image_out");
  pub->publish(*msg);
}

void
Consumer::description_in_callback(std_msgs::msg::String::SharedPtr msg)
{
  (void)msg;
  RCLCPP_INFO(get_logger(), "String message received");
}
```
### Deployment with ROSSDL

In the deployment phase, we define one or more **systems**, each one will be a launcher file that will be generated in the `install` directory, right in the correct location. A `.rossystem` file written in Yaml define a system complete, selecting nodes to deploy, their connections, and parameters. This is an example of two systems in the same file:

```
---
rossdl_simple_test:
  fromGitRepo: "https://github.com/jane-doe/project_example.git:branch"
  systems:
    system_1:
      nodes: [rossdl_simple_test::image_filter, rossdl_simple_test::consumer]
      connections:
        - [/image_filter/image_out, /consumer/image_in]
        - [/image_filter/description_out, /consumer/description_in]
        - [/camera/rgb/image_raw, /image_filter/image_in]
        - [/consumer/image_out, /other_node/image_in]
      parameters:
        - [/image_filter/description_label, 'image raw']
        - [/*/use_sim_time, True]
    system_2:
      nodes: [rossdl_simple_test::image_filter, rossdl_simple_test::consumer]
      connections:
        - [/image_filter/image_out, /consumer/image_in]
        - [/image_filter/description_out, /consumer/description_in]
      parameters:
        - [/image_filter/description_label, 'image compressed']
        - [/*/use_sim_time, False]   
```

When building the system, launchers for both systems can be created. For example, the generated file `system_1.launcher.py` (in `install/rossdl_simple_test/share/rossdl_simple_test/launch/`) will have code like this:

```
    load_composable_nodes = LoadComposableNodes(
        target_container=container_name,
            composable_node_descriptions=[
                ComposableNode(
                    package = 'rossdl_simple_test',
                    plugin = 'rossdl_simple_test::ImageFilter',
                    name = 'image_filter',
                    remappings = [
                        ('/image_filter/image_out', '/consumer/image_in'),
                        ('/image_filter/description_out', '/consumer/description_in'),
                        ('/image_filter/image_in', '/camera/rgb/image_raw'),
                    ],
                    parameters=[{
                        'description_label': 'image raw',
                        'use_sim_time': True,
                    }],
                ),
                ComposableNode(
                    package = 'rossdl_simple_test',
                    plugin = 'rossdl_simple_test::Consumer',
                    name = 'consumer',
                    remappings = [
                        ('/consumer/image_out', '/other_node/image_in'),
                    ],
                    parameters=[{
                        'use_sim_time': True,
                    }],
                ),
    ])
```

## Further documentation

* More examples: https://github.com/CoreSenseEU/rossdl/tree/rolling/rossdl_tests
* More documentation:  Coming soon at https://coresenseeu.github.io/

## Usage

Just place a definition file in your package and add this to CMakeLists:

```
find_package(rossdl_cmake REQUIRED)

rossdl_generate_code(${PROJECT_NAME}
  "description.ros2"
  ${dependencies}
)

rossdl_generate_system(
  "description.rossystem"
  "system_1"
  "system_2"
)
```

and compile with colcon:

```
colcon build --symlink-install
```

Package `rossdl_test` contains an example of use.

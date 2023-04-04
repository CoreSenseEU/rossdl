# rossdl (ROS System Definition Language)

This repo contains a code generator that automatically generate base class for ROS2 nodes.
It starts from a file like this:

```
---
rossdl_test:
  FromGitRepo: "https://github.com/jane-doe/project_example.git:branch"
  artifacts:
    image_filter:
      node: "image_filter"
      publishers:
        image_out:
          type: "sensor_msgs/msg/Image"
          qos:
            qos_profile: "sensor_qos"
        description_out:
          type: "std_msgs/msg/String"
          qos:
            qos_history_depth: 100
      subscribers:
        image_in:
          type: "sensor_msgs/msg/Image"
          qos:
            qos_profile: "sensor_qos"
            qos_reliability: "reliable"
        laser_in:
          type: "sensor_msgs/msg/LaserScan"
          qos:
            qos_profile: "sensor_qos"
            qos_reliability: "reliable"
      parameters:
        description_label:
          type: string
          default: "default image"
    consumer:
      node: "consumer"
      subscribers:
        image_in:
          type: "sensor_msgs/msg/Image"
          qos:
            qos_profile: "sensor_qos"
            qos_reliability: "reliable"
        description_in:
          type: "std_msgs/msg/String"
```

## Usage

Just place a definition file in your package, and add this to CMakeLists:

```
rossdl_generate_code(${PROJECT_NAME}
  "description.sdl"
  ${dependencies}
)
```

Package `rossdl_test` contains an example of use.

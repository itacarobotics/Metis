# How to navigate
### /src/deltarobot
Middle-ware nodes and classes, runs ROS2 (on laptop)
- configuration file
- robot controller (node)
    - delta robot (class)
        - inverse geometry (class)
        - trajectroy generator (class)
- task scheduler (node)

### /src/deltarobot_inputs
High-level nodes to interact with the robot, runs ROS2 (on laptop)
- graphical user interface (node)
- gcode parser (node)
- template (node), for adding new features

### /src/firmware_esp32
Low-level firmware, runs 
[micro-ROS](https://github.com/micro-ROS/micro_ros_platformio) 
(on ESP32 microcontroller)
- micro-ROS init
- callback functions
- stepper motors driver

### /src/deltarobot_description
URDF description of the robot
- complete robot description (for visualizer)
- single closed chain description (for inverse geometry)

### /src/deltarobot_utils
Deprecated as last update.

### /src/micro_ros_setup
Used to setup the micro-ROS environment 
([help](https://github.com/micro-ROS/micro_ros_platformio))
(on ESP32 microcontroller)

### /src/uros
The actual micro-ROS agent 
([help](https://github.com/micro-ROS/micro_ros_platformio))
(on ESP32 microcontroller)
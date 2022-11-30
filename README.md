# ENPM808X - Assignment : ROS2 Tutorials

## Overview and Description

A basic Publisher/Subscriber package for ROS2 written in C++

## License

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

## Author

Tharun V. Puthanveettil

## Assumptions
ROS2 Humble package is created and tested on ubuntu 20.02 (Linux).
The colcon build is used for building the package.

## Dependencies
ROS2 Humble has to be build and sourced

## To build the package

Open new terminal window and type the following:
```
mkdir ros2_ws/src
cd ros2_ws/src
git clone https://github.com/tvpian/beginner_tutorials.git
cd ..
source <path to ros2 setup>/install/setup.bash    
colcon build
source install/setup.bash
```
Open a new terminal window and type:
```
cd ros2_ws
source <path to ros2 setup>/install/setup.bash    
source install/setup.bash
ros2 run first_publisher publisher_lambda
```
Open a new terminal window and type:
```
cd ros2_ws
source <path to ros2 setup>/install/setup.bash    
source install/setup.bash
ros2 run first_subscriber subscriber_lambda
```
To end the process, type ctrl+C on all the terminal windows one by one.


## To test the server
```
cd ros2_ws
source <path to ros2 setup>/install/setup.bash    
source install/setup.bash
ros2 run first_publisher publisher_lambda
```
Open a new terminal window and type:
```
cd ros2_ws
source <path to ros2 setup>/install/setup.bash    
source install/setup.bash
ros2 service call /change_msg custom_services/srv/PassMsg "{msg: Hello Terps..}" 
```
The output of the publisher can be seen changed from "Hello USA" to "Hello Terps.."

To end the process, type ctrl+C on all the terminal windows one by one.



## To test the launch file
```
cd ros2_ws
source <path to ros2 setup>/install/setup.bash    
source install/setup.bash
ros2 launch src/beginner_tutorials/src/launch/tester_launch.yaml freq:=20.0 topic_name:="getitout"
```
Both the publisher and subscriber nodes are invoked with the parameters - freq & topic_name

To end the process, type ctrl+C on all the terminal windows one by one.

## To run Cpplint
```
cd ros2_ws
run_cpplint.sh
```

## To run Cppcheck
```
cd ros2_ws
run_cppcheck.sh
```

## Logging
**Make sure roscore is running**
* Invoke rqt console GUI
```
rqt_console
```
* To check the results of rqt_log
```
cd ros2_ws/results
view rqt_log_result1.png
view rqt_log_result2.png
```


## To test rosbag output:
### Replay rosbag recording:
* Open terminal window and type:
    > ros2 bag play results/rosbag_recording
* Open a new terminal window and type
    > ros2 run first_subscriber subscriber_lambda

* To view rosbag output info:
    > ros2 bag info results/rosbag_recording
    - output:
    ```
    Files:             rosbag_recording_0.db3
    Bag size:          135.0 KiB
    Storage id:        sqlite3
    Duration:          40.629s
    Start:             Nov 30 2022 10:58:43.964 (1669823923.964)
    End:               Nov 30 2022 10:59:24.593 (1669823964.593)
    Messages:          479
    Topic information: Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 1 | Serialization Format: cdr
                       Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 396 | Serialization Format: cdr
                       Topic: /topic_old | Type: std_msgs/msg/String | Count: 82 | Serialization Format: cdr
                       Topic: /parameter_events | Type: rcl_interfaces/msg/ParameterEvent | Count: 0 | Serialization Format: cdr
    ```
    Note: '/topic_old' is the topic to which publisher is publishing instead of '/chatter'


## To test tf2
    
    ros2 run first_publisher publisher_lambda talk 0 1 0 1 0 1
    
    - publishing a static tranform between frame 'talk' and 'world' with a translation of '0 1 0' and rotation of '1 0 1'
  
Open a new terminal window and type:

    ros2 run tf2_ros tf2_echo world talk
    
    - output:
    ```
    At time 0.0
    - Translation: [0.000, 1.000, 0.000]
    - Rotation: in Quaternion [0.421, 0.230, 0.421, 0.770]
    ```



## To perform Gtest
```
cd ros2_ws
source <path to ros2 setup>/install/setup.bash    
source install/setup.bash 
ros2 run first_publisher publisher_lambda talk 0 1 0 1 0 1
```
Note: As the designed Gtest is for testing the server block ensure the server is started.

Output without verbose:
```
colcon test --packages-select minimal_integration_test
```
Output with verbose:
```
colcon test --event-handlers console_direct+ --packages-select minimal_integration_test
```
To end the process, type ctrl+C on all the terminal windows one by one.

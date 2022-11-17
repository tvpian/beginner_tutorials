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
ros2 launch src/launch/tester_launch.yaml freq:=20.0 topic_name:="getitout"
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

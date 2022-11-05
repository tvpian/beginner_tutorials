# ENPM808X - Week 9 Assignment : ROS2 Package for simple Publisher / Subscriber

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
// "Copyright[2022] by Tharun V. Puthanveettil"

#ifndef TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP
#define TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>


#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 0
#define LEFT   1
#define RIGHT  2

#define LINEAR_VELOCITY  0.3
#define ANGULAR_VELOCITY 1.5

#define GET_TB3_DIRECTION 0
#define TB3_DRIVE_FORWARD 1
#define TB3_RIGHT_TURN    2
#define TB3_LEFT_TURN     3


/**
 * @brief A class that implements the roomba walker algorithm
 *
 * 
 */
class roomba : public rclcpp::Node {
 public:
  roomba();
  ~roomba();

 private:
  // ROS topic publishers
  /**
   * @brief Publishes the velocity commands to the turtlebot3
   *
   * 
   */
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  /**
   * @brief Subscribes to the laser scan data from the turtlebot3
   * 
   */
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  /**
   * @brief Subscribes to the odometry data from the turtlebot3
   * 
   * 
   */
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Variables to record the current pose and previous pose of the turtlebot3
  double robot_pose_;
  double prev_robot_pose_;
  double scan_data_[3];

  /**
   * @brief Timer to update the robot pose and scan data
   * 
   */
  rclcpp::TimerBase::SharedPtr update_timer_;

  /**
   * @brief Callback function for the laser scan data
   * 
   *
   */
  void update_callback();
  /**
   * @brief Callback function for the odometry data
   * 
   * @param linear // Linear velocity
   * @param angular // Angular velocity
   */
  void update_cmd_vel(double linear, double angular);
  /**
   * @brief Callback function for the laser scan data
   * 
   * @param msg // Laser scan data
   */
  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  /**
   * @brief 
   * 
   * @param msg // Odometry data
   */
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};
#endif // TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP

// Copyright (c) 2022 by Tharun V. Puthanveettil
#include <memory>

#include "custom_services/srv/pass_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

// parameter types
using PARAMETER_EVENT = std::shared_ptr<rclcpp::ParameterEventHandler>;
using PARAMETER_HNADLE = std::shared_ptr<rclcpp::ParameterCallbackHandle>;

/**
 * @brief Sample class to demonstrate the use of Ros2 publisher and server node
 * simultaneously
 *
 */
class MyFirstPubSev : public rclcpp::Node {
 public:
  explicit MyFirstPubSev(char *transformations[])
      : Node("my_first_publisher"), count_(0) {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
    param_desc.description = "Set callback frequency.";
    this->declare_parameter("freq", 2.0, param_desc);  // default =  2 hz
    auto param = this->get_parameter("freq");
    auto freq = param.get_parameter_value().get<std::float_t>();

    auto param_desc2 = rcl_interfaces::msg::ParameterDescriptor();
    param_desc2.description = "Set topic name.";
    this->declare_parameter("topic_name", "topic_old",
                            param_desc2);  // default =  "topic_old"
    auto param2 = this->get_parameter("topic_name");
    auto topicname = param2.get_parameter_value().get<std::string>();

    tf_static_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Publish static transforms once at startup

    this->make_transforms(transformations);

    publisher_ = this->create_publisher<std_msgs::msg::String>(topicname, 10);
    timer_ = this->create_wall_timer(
        std::chrono ::milliseconds(static_cast<int>((1000 / freq))),
        std::bind(&MyFirstPubSev::publisher_timer_callback, this));
    RCLCPP_INFO_STREAM(this->get_logger(), "Publisher has been started..");

    server_ = this->create_service<custom_services::srv::PassMsg>(
        "change_msg", std::bind(&MyFirstPubSev::server_callback, this,
                                std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO_STREAM(this->get_logger(), "Server has been started..");
  }

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<custom_services::srv::PassMsg>::SharedPtr server_;
  std::string test_msg = "Hello USA!!";
  int count_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  char *transformations[];

  /**
   * @brief Callback function for the publisher timer
   *
   */
  void publisher_timer_callback() {
    auto message = std_msgs::msg::String();
    geometry_msgs::msg::TransformStamped t;
    message.data = test_msg;
    count_++;
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "I published: '%s'", message.data.c_str());
    if (count_ >= 5) {
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Tesing Logging(Debug)");
    }
    if (count_ >= 10) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Tesing Logging(Info)");
    }
    if (count_ >= 15) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Tesing Logging(Warn)");
    }
    if (count_ >= 20) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Tesing Logging(Error)");
    }
    if (count_ >= 25) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "Tesing Logging(Fatal)");
    }
  }

  /**
   * @brief Callback function for the server
   *
   * @param new_msg
   */
  void change_string(std::string new_msg) { test_msg = new_msg; }

  /**
   * @brief Callback function for the server
   *
   * @param request
   * @param response
   */
  void server_callback(
      const std::shared_ptr<custom_services::srv::PassMsg::Request> request,
      std::shared_ptr<custom_services::srv::PassMsg::Response> response) {
    response->output = request->msg;
    change_string(request->msg);
    RCLCPP_WARN_STREAM(
        this->get_logger(),
        "Incoming request for changing message to" << request->msg);
  }

  /**
   * @brief Function to publish static transforms
   *
   * @param transformation
   */
  void make_transforms(char *transformation[]) {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = transformation[1];

    t.transform.translation.x = atof(transformation[2]);
    t.transform.translation.y = atof(transformation[3]);
    t.transform.translation.z = atof(transformation[4]);
    tf2::Quaternion q;
    q.setRPY(atof(transformation[5]), atof(transformation[6]),
             atof(transformation[7]));
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(t);
  }
};

/**
 * @brief Main function to run the node
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv) {
  auto logger = rclcpp::get_logger("logger");

  // Obtain parameters from command line arguments
  if (argc != 8) {
    RCLCPP_INFO(logger,
                "Invalid number of parameters\nusage: "
                "$ ros2 run first_publisher publisher_lambda "
                "child_frame_name x y z roll pitch yaw");
    return 1;
  }

  // As the parent frame of the transform is `world`, it is
  // necessary to check that the frame name passed is different
  if (strcmp(argv[1], "world") == 0) {
    RCLCPP_INFO(logger, "Your child frame cannot be 'world'");
    return 1;
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyFirstPubSev>(argv);
  // Create a shared pointer
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

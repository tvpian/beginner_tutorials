// Copyright (c) 2022 by Tharun V. Puthanveettil
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


// parameter types
using PARAMETER_EVENT  = std::shared_ptr<rclcpp::ParameterEventHandler>;
using PARAMETER_HNADLE = std::shared_ptr<rclcpp::ParameterCallbackHandle>;


/**
 * @brief Sample class to demonstrate the use of Ros2 subscriber
 * 
 */
class MinimalSubscriber : public rclcpp::Node {
 public:
  MinimalSubscriber() : Node("minimal_subscriber") {
    auto param_desc2 = rcl_interfaces::msg::ParameterDescriptor();
    param_desc2.description = "Set topic name.";
    this->declare_parameter("topic_name",
    "topic_old", param_desc2);  // default =  "topic_old"
    auto param2 = this->get_parameter("topic_name");
    auto topicname = param2.get_parameter_value().get<std::string>();
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        topicname, 10, [this](std_msgs::msg::String::UniquePtr msg) {
          RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
        });
  }

 private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

/**
 * @brief Main function to run the subscriber node
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

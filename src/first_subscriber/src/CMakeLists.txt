find_package(ament_cmake REQUIRED)
add_executable(subscriber_lambda lambda.cpp)
ament_target_dependencies(subscriber_lambda rclcpp std_msgs)


install(TARGETS
  subscriber_lambda
  DESTINATION lib/${PROJECT_NAME})

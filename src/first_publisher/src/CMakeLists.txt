find_package(ament_cmake REQUIRED)
add_executable(publisher_lambda lambda.cpp)
ament_target_dependencies(publisher_lambda rclcpp std_msgs)



install(TARGETS
  publisher_lambda
  DESTINATION lib/${PROJECT_NAME}
)


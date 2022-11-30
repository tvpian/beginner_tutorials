find_package(ament_cmake REQUIRED)
find_package(custom_services REQUIRED)

add_executable(publisher_lambda lambda.cpp)
ament_target_dependencies(
	publisher_lambda 
	rclcpp 
	std_msgs 
	custom_services
	geometry_msgs
    rclcpp
   	tf2
   	tf2_ros
   )



install(TARGETS
  publisher_lambda
  DESTINATION lib/${PROJECT_NAME}
)


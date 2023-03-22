cmake_minimum_required(VERSION 3.8)
project(rrbot_gazebo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rob_kinematics_server REQUIRED)
find_package(Armadillo REQUIRED)



add_executable(publisher src/position_publisher.cpp)
 ament_target_dependencies(publisher rclcpp std_msgs)

 add_executable(forward_kinematics_scara src/forward_kinematics.cpp)
 ament_target_dependencies(forward_kinematics_scara rclcpp std_msgs geometry_msgs tf2_geometry_msgs) 

 add_executable(server src/inv_kinematics.cpp)
 ament_target_dependencies(server rclcpp rob_kinematics_server)

 add_executable(client src/inv_kinematics_client.cpp)
 ament_target_dependencies(client rclcpp rob_kinematics_server)

 add_executable(switch src/controller_switch.cpp)
 ament_target_dependencies(switch rclcpp controller_manager_msgs)

 add_executable(reference_joint_position_server src/position_reference_service.cpp)
 ament_target_dependencies(reference_joint_position_server rclcpp rob_kinematics_server)

 add_executable(joint_position_controller src/position_controller.cpp)
 ament_target_dependencies(joint_position_controller rclcpp rob_kinematics_server std_msgs sensor_msgs)

 add_executable(velocity_jacobian_services src/velocity_jacobian_service.cpp)
 target_link_libraries(velocity_jacobian_services ${ARMADILLO_LIBRARIES})
 ament_target_dependencies(velocity_jacobian_services rclcpp rob_kinematics_server std_msgs sensor_msgs geometry_msgs tf2_geometry_msgs Armadillo)

install(TARGETS
   publisher
   forward_kinematics_scara
   server
   client
   switch
   reference_joint_position_server
   joint_position_controller
   velocity_jacobian_services
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

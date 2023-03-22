#include "rclcpp/rclcpp.hpp"
#include "rob_kinematics_server/srv/inverse_kinematics.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 4) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: 3-D coordinate values of SCARA bot end effector: X Y Z");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("inverse_kinematics");
  rclcpp::Client<rob_kinematics_server::srv::InverseKinematics>::SharedPtr client =
    node->create_client<rob_kinematics_server::srv::InverseKinematics>("joint_values");

  auto request = std::make_shared<rob_kinematics_server::srv::InverseKinematics::Request>();
  request->end_effector_pose[0] = std::stof(argv[1]);
  request->end_effector_pose[1] = std::stof(argv[2]);
  request->end_effector_pose[2] = std::stof(argv[3]);


  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }


  

  auto result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS){
    std::array<double, 3> test = result.get()->joint_values;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "response [  %0.2f, %0.2f, %0.2f] ",  test[0], test[1], test[2]);// "response: [%f, %f, %f]", result.get()->joint_values[0], result.get()->joint_values[1], result.get()->joint_values[2]);
  }
  else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}
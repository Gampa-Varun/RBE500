#include "rclcpp/rclcpp.hpp"
#include "rob_kinematics_server/srv/reference.hpp"
#include <cmath>
#include <iostream> 
#include <functional>

#include <memory>

void referenceValues(const std::shared_ptr<rob_kinematics_server::srv::Reference::Request> request,
          std::shared_ptr<rob_kinematics_server::srv::Reference::Response>      response, float ref_values[])
{
  if(request->need_ref){

  response->joint_values[1] =  ref_values[1];

  //Given theta3 and xc,yc,zc , we can get theta3

  response->joint_values[0] =ref_values[0];

  response->joint_values[2] = ref_values[2];

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request \n");
  
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%0.2f, %0.2f, %0.2f]", response->joint_values[0], response->joint_values[1], response->joint_values[2]);
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
    if (argc != 4) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: 3-D coordinate values of SCARA bot end effector: X Y Z");
      return 1;
  }


  float ref_values[3];

  ref_values[0] = std::stof(argv[1]);
  ref_values[1] = std::stof(argv[2]);
  ref_values[2] = std::stof(argv[3]);

  using namespace std::placeholders;
  std::function<void(const std::shared_ptr<rob_kinematics_server::srv::Reference::Request>, std::shared_ptr<rob_kinematics_server::srv::Reference::Response>)> refered_f = std::bind(referenceValues,_1,_2,ref_values);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("reference_position_server");

  rclcpp::Service<rob_kinematics_server::srv::Reference>::SharedPtr service =
    node->create_service<rob_kinematics_server::srv::Reference>("reference_joint_values", refered_f);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to send joint values.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
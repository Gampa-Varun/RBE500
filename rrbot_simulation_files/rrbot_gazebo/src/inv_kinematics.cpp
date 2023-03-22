#include "rclcpp/rclcpp.hpp"
#include "rob_kinematics_server/srv/inverse_kinematics.hpp"
#include <cmath>
#include <iostream> 

#include <memory>

void invKinematics(const std::shared_ptr<rob_kinematics_server::srv::InverseKinematics::Request> request,
          std::shared_ptr<rob_kinematics_server::srv::InverseKinematics::Response>      response)
{

  float x_c = request->end_effector_pose[0];
  float y_c = request->end_effector_pose[1];
  float z_c = request->end_effector_pose[2];

  //Theta 1 is nothing but tan inverse of yc and xc



  float r = x_c;
  //IMPORTANT: s = d[0]-z_c as, as per the DH frames chosen, the downward direction is positive y direction.
  float s = y_c;

  //Obtaining cos (theta3) which is denoted as D
  float D = (x_c*x_c + y_c*y_c - 1 - 1)/(2);

  //Obtaining theta3 from cos(theta3) value
  response->joint_values[1] =  atan2(sqrt(1-D*D),D);

  //Given theta3 and xc,yc,zc , we can get theta3

  response->joint_values[0] = atan2(s,r) - atan2(sin(response->joint_values[1]),1+cos(response->joint_values[1]));

  response->joint_values[2] = 1- z_c;

  std::cout<<response->joint_values[0]<<std::endl;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nx: %f" " y: %f" " z: %f",
                x_c, y_c, z_c);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%0.2f, %0.2f, %0.2f]", response->joint_values[0], response->joint_values[1], response->joint_values[2]);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("rob_kinematics_server");

  rclcpp::Service<rob_kinematics_server::srv::InverseKinematics>::SharedPtr service =
    node->create_service<rob_kinematics_server::srv::InverseKinematics>("joint_values", &invKinematics);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to compute joint values.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
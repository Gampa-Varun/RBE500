#include "rclcpp/rclcpp.hpp"
#include "rob_kinematics_server/srv/reference.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;





class InputjointSubscriber : public rclcpp::Node
{
  public:

    //Defining the subscriber valraibles
    //std::function<void(std::array<double, 3>)> fcn = std::bind(&InputjointSubscriber::forward_callback, this, _1);
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    size_t count_;

    // Reference value
    std::array<double, 3> ref;


    InputjointSubscriber(std::array<double, 3> newref)
    : Node("kinematics"), count_(0), ref(newref)
    {
      // Biniding the subscriber call to callback functions (forward and inverse for forward and inverse kinematics)
      // Setting the pointer to subscriber calls

      //Subscribing to input_joint_angles for forward kinematics
      subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&InputjointSubscriber::forward_callback, this, _1));

      publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_effort_controller/commands", 10);
      timer_ = this->create_wall_timer(
      10ms, std::bind(&InputjointSubscriber::timer_callback, this));
    }

  private:

    // This call back converts input joint angles to give the homogenous transformation matrix which contains the end effector pose 
    // in the final column
    void forward_callback(const sensor_msgs::msg::JointState & msg) 
    {


      float Kp[3] = {1.25,2.5,100};

      float Kd[3] = {1.7,2,50};


      for (int i = 0; i < 3; ++i) {

        error[i] = ref[i]-msg.position[i];

        error_dev[i] = (error[i] - error_prev[i])/0.01;

        effort[i] = Kp[i]*error[i] + Kd[i]* error_dev[i] ;

        error_prev[i] = error[i];


      }
      effort[2] = effort[2]  -9.8002347946167;







    }

    void timer_callback()
    {
      auto message = std_msgs::msg::Float64MultiArray();
      message.data.push_back(effort[0]);
      message.data.push_back(effort[1]);
      message.data.push_back(effort[2]);
      RCLCPP_INFO(this->get_logger(), "Publishing...");
      publisher_->publish(message);
    }




    //Defining the system parameters
    float a[3] = {1,1,0};
    float alpha[3] = {0,3.14,0};
    float d[3] = {2,0,1};
    float theta[3] = {0,0,0};
    float error[3] = {0,0,0};
    float effort[3] = {0,0,0};
    float error_dev[3] = {0,0,0};
    float error_prev[3] = {0,0,0};
    float error_sum[3] = {0,0,0};


  
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("Reference_joint_values_receiver");
  rclcpp::Client<rob_kinematics_server::srv::Reference>::SharedPtr client =
    node->create_client<rob_kinematics_server::srv::Reference>("reference_joint_values");

  auto request = std::make_shared<rob_kinematics_server::srv::Reference::Request>();
  request->need_ref = true;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);

  std::array<double, 3> test;

  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS){
    test = result.get()->joint_values;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "response [  %0.2f, %0.2f, %0.2f] ",  test[0], test[1], test[2]);// "response: [%f, %f, %f]", result.get()->joint_values[0], result.get()->joint_values[1], result.get()->joint_values[2]);
  }

  if(request->need_ref)
    rclcpp::spin(std::make_shared<InputjointSubscriber>(test));


  rclcpp::shutdown();
  return 0;
}
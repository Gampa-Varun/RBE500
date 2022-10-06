#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/quaternion.hpp" 
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


using std::placeholders::_1;
using namespace std::chrono_literals;

class EulerSubscriber : public rclcpp::Node
{
  public:
    EulerSubscriber()
    : Node("euler_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "euler_angles", 10, std::bind(&EulerSubscriber::topic_callback, this, _1));

      //publisher_ = this->create_publisher<geometry_msgs::msg::Quaternion>("topic", 10);
      //timer_ = this->create_wall_timer(
      //1000ms, std::bind(&EulerSubscriber::timer_callback, this));

    }

  private:
    void topic_callback(const std_msgs::msg::Float32MultiArray & msg)
    {

      auto quat1_g = geometry_msgs::msg::Quaternion();
      auto quat2_g = geometry_msgs::msg::Quaternion();
      auto quat3_g = geometry_msgs::msg::Quaternion();

      auto quat1 = tf2::Quaternion();
      auto quat2 = tf2::Quaternion();
      auto quat3 = tf2::Quaternion();
      

      quat1_g.x = 0;
      quat1_g.y = 0;
      quat1_g.z = sin(msg.data[0]/2); 
      quat1_g.w = cos(msg.data[0]/2);

      quat2_g.x = 0;
      quat2_g.y = sin(msg.data[1]/2);
      quat2_g.z = 0; 
      quat2_g.w = cos(msg.data[1]/2);

      quat3_g.x = 0;
      quat3_g.y = 0;
      quat3_g.z = sin(msg.data[2]/2); 
      quat3_g.w = cos(msg.data[2]/2);

      tf2::convert(quat1_g, quat1);
      tf2::convert(quat2_g, quat2);
      tf2::convert(quat3_g, quat3);

      quat = quat1*quat2*quat3;

      /*float q1 = cos(msg.data[2]/2)*cos(msg.data[1]/2)*cos(msg.data[0]/2) + sin(msg.data[2]/2)*sin(msg.data[1]/2)*sin(msg.data[0]/2);
      float q2 = sin(msg.data[2]/2)*cos(msg.data[1]/2)*cos(msg.data[0]/2) - cos(msg.data[2]/2)*sin(msg.data[1]/2)*sin(msg.data[0]/2);
      float q3 = cos(msg.data[2]/2)*sin(msg.data[1]/2)*cos(msg.data[0]/2) + sin(msg.data[2]/2)*cos(msg.data[1]/2)*sin(msg.data[0]/2);
      float q4 = cos(msg.data[2]/2)*cos(msg.data[1]/2)*sin(msg.data[0]/2) - sin(msg.data[2]/2)*sin(msg.data[1]/2)*cos(msg.data[0]/2);

      float c = q1*q1 + q2*q2 + q3*q3 + q4*q4;*/

      RCLCPP_INFO(this->get_logger(), "'%lf' ", quat.w());
      RCLCPP_INFO(this->get_logger(), "'%lf' ", quat.x());
      RCLCPP_INFO(this->get_logger(), "'%lf' ", quat.y());
      RCLCPP_INFO(this->get_logger(), "'%lf' ", quat.z());
      
     // RCLCPP_INFO(this->get_logger(), "'%lf' ", c);
     // RCLCPP_INFO(this->get_logger(), "'%lf' ", q1);
     // RCLCPP_INFO(this->get_logger(), "'%lf' ", q2);
     // RCLCPP_INFO(this->get_logger(), "'%lf' ", q3);
     // RCLCPP_INFO(this->get_logger(), "'%lf' ", q4);

    

      
    }

  /*void timer_callback()
  {
    auto message = geometry_msgs::msg::Quaternion();
    message = tf2::toMsg(quat);;
    publisher_->publish(message);
  }*/
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    //rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr publisher_;
    tf2::Quaternion quat;
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EulerSubscriber>());
  rclcpp::shutdown();
  return 0;
}

#include "rclcpp/rclcpp.hpp"
#include "rob_kinematics_server/srv/velreference.hpp"
#include "rob_kinematics_server/srv/jointvel.hpp"
#include <cmath>
#include <iostream> 
#include <functional>
#include <string>
#include <memory>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/quaternion.hpp" 
#include "geometry_msgs/msg/pose.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#define _USE_MATH_DEFINES
#include <cmath>
#include <armadillo>
#include "sensor_msgs/msg/joint_state.hpp"


arma::mat zv(3,3,arma::fill::zeros) ;
arma::mat ov(3,3,arma::fill::zeros) ;
arma::mat Jacobian(6,3,arma::fill::zeros);
arma::vec desired_joint_velocities(3);
bool ref_received = false;
arma::colvec end_effector_velocities(6);
arma::vec end_vel(6);

arma::vec joint_velo(3);


using std::placeholders::_1;
using namespace std::chrono_literals;

void endeffectorValues2(arma::vec joint_vel)
{

  

  Jacobian.rows(0,2).col(0) = arma::cross(zv.col(0),ov.col(0));
  Jacobian.rows(0,2).col(1) = arma::cross(zv.col(1),ov.col(1));
  Jacobian.rows(0,2).col(2) = zv.col(2);
  Jacobian.rows(3,5).col(0) = zv.col(0);
  Jacobian.rows(3,5).col(1) = zv.col(1);

  end_vel = Jacobian*joint_vel;



}


void jointValues2(arma::vec endeffector_velocities)
{






  arma::mat Jacobian_inverse = arma::pinv(Jacobian); 

  arma::mat semi_Jacobian(3,3);

  semi_Jacobian.row(0) = Jacobian.row(0);
  semi_Jacobian.row(1) = Jacobian.row(1);
  semi_Jacobian.row(2) = Jacobian.row(2);

  float epsilon = 0.0001;
  for (int i = 0; i < 3; ++i)
  {
    semi_Jacobian.col(i) = semi_Jacobian.col(i) + (i+1)*epsilon;
  }

  arma::mat semi_Jacobian_inv = arma::inv(semi_Jacobian);

  arma::vec end_effector_xy(3);

  end_effector_xy(0) = end_effector_velocities(0);
  end_effector_xy(1) = end_effector_velocities(1);
  end_effector_xy(2) = end_effector_velocities(2);



  // Jacobian.print("Jacobian: ");
  // zv.print("z: ");

  


  /*for(int i =0;i<3;i++){
    desired_joint_velocities(i) = Jacobian_inverse(i,0)*endeffector_velocities(0) + Jacobian_inverse(i,1)*endeffector_velocities(1) + Jacobian_inverse(i,2)*endeffector_velocities(2) + Jacobian_inverse(i,3)*endeffector_velocities(3) + Jacobian_inverse(i,4)*endeffector_velocities(4)+ Jacobian_inverse(i,5)*endeffector_velocities(5) ;
  }
*/

  desired_joint_velocities = semi_Jacobian_inv*end_effector_xy;

  /*arma::mat check2 = semi_Jacobian*desired_joint_velocities;

  endeffector_velocities.print("check3: ");

  check2.print("check2: ");

  desired_joint_velocities.print("check5: ");
*/




}


// Defining the subscriber class which contains the set of subscribers required for kinematics
class InputjointSubscriberforvelocity : public rclcpp::Node
{
  public:
    InputjointSubscriberforvelocity()
    : Node("velocitykinematics"), count_(0)
    {
      
      // Biniding the subscriber call to callback functions (forward and inverse for forward and inverse kinematics)
      // Setting the pointer to subscriber calls

      //Subscribing to input_joint_angles for forward kinematics
      subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&InputjointSubscriberforvelocity::forward_callback, this, _1));

      publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_effort_controller/commands", 10);
      timer_ = this->create_wall_timer(
      10ms, std::bind(&InputjointSubscriberforvelocity::timer_callback, this));

      publisher2_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_velocities_references", 10);

    /*  timer2_ = this->create_wall_timer(
      10ms, std::bind(&InputjointSubscriberforvelocity::timer_callback2, this));*/


    }

  private:

    //Function for matrix multiplication
    std::vector<std::vector<float>> multiply_small(std::vector<std::vector<float>> v1 ,std::vector<std::vector<float>> v2 ){
        int nrows;
        nrows = v1.size();
        int ncolumns;
        ncolumns = v2[0].size();
        int sum_over;
        sum_over = v1[0].size();
        assert(v1[0].size() == v2.size());
        std::vector<std::vector<float>> vres(nrows,std::vector<float>(ncolumns)) ;
        for (int i = 0; i < nrows; ++i)
        {
          for(int j = 0; j<ncolumns;++j){
            float sum = 0;
            for(int k =0;k<sum_over;++k){

              sum = sum+ v1[i][k]*v2[k][j];
            }
            vres[i][j] = sum;
          }
        }
        return vres;
    }

    //Funcion which creates the DH matrix from the values of theta, alpha, a ,d
    std::vector<std::vector<float>> DH_matrix( float theta , float alpha, float a, float d  ){
        std::vector<std::vector<float>> Ai(4,std::vector<float>(4));
        Ai= {{cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)}, {sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)}, {0, sin(alpha), cos(alpha), d}, {0, 0, 0, 1}};
        return Ai;
    }    



    // This call back converts input joint angles to give the homogenous transformation matrix which contains the end effector pose 
    // in the final column
    void forward_callback(const sensor_msgs::msg::JointState & msg) 
    {
      //Obtaining the theta values from the Float32MultiArray msg
      float rz, ry,rz_2;
      theta[0] = msg.position[0];
      theta[1] = msg.position[1];
      d[2] = msg.position[2]+0.5;
     
      std::vector<std::vector<float>> Ai(4,std::vector<float>(4));
      std::vector<std::vector<float>> H(4,std::vector<float>(4)); 
      H = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
      
      
  
      //Creating Ai matrices and multiplying them to the Homogenous matrices
      for (int i = 0; i < 3; ++i)
      {
        for (int j = 0; j < 3; j++)
        {
          ov(j,i) = H[j][3];
          zv(j,i) = H[j][2];
        }
        Ai = DH_matrix(theta[i],alpha[i],a[i],d[i]);
        H = multiply_small(H,Ai);
      }

      for(int i =0; i<2;i++){
        for (int j = 0; j < 3; j++)
          ov(j,i) = H[j][3]-ov(j,i);
      }

      

      for (int i = 0; i < 3; i++)
      {
        joint_velo(i) = msg.velocity[i];
      }

      if(ref_received){
        jointValues2(end_effector_velocities);
        endeffectorValues2(joint_velo);
        joint_velo.print("joint_velocity: ");
        end_vel.print("end_vel: ");
        desired_joint_velocities.print("desired joint velocity: ");
        std::cout<<"error_0: "<<error[0]<<std::endl;

      }

      
      

      float Kp[3] = {15,6,1};

      float Kd[3] = {0.15,0.2,1};

      if (ref_received){
      
        for (int i = 0; i < 3; ++i) {

          error[i] = desired_joint_velocities(i)-joint_velo(i);

          error_dev[i] = (error[i] - error_prev[i])/0.01;

          effort[i] = Kp[i]*error[i] + Kd[i]* error_dev[i] ;

          error_prev[i] = error[i];

        }
      }
      
      


    }

    void timer_callback()
    {
      auto message = std_msgs::msg::Float64MultiArray();

      message.data.push_back(effort[0]);
      message.data.push_back(effort[1]);
      message.data.push_back(effort[2]);
      //RCLCPP_INFO(this->get_logger(), "Publishing...");
      if(ref_received){
          publisher_->publish(message);
        }

      auto message2 = std_msgs::msg::Float64MultiArray();

      message2.data.push_back(desired_joint_velocities(0));
      message2.data.push_back(desired_joint_velocities(1));
      message2.data.push_back(desired_joint_velocities(2));
      message2.data.push_back(joint_velo(0));
      message2.data.push_back(joint_velo(1));
      message2.data.push_back(joint_velo(2));
      message2.data.push_back(end_vel(1));

      //RCLCPP_INFO(this->get_logger(), "Publishing...");
      if(ref_received){
          publisher2_->publish(message2);
      }
    }

  /*  void timer_callback2()
    {
      auto message = std_msgs::msg::Float64MultiArray();

      message.data.push_back(end_vel(0));
      message.data.push_back(end_vel(1));
      message.data.push_back(end_vel(2));
      //RCLCPP_INFO(this->get_logger(), "Publishing...");
      if(ref_received){
          publisher_->publish(message);
      }
    }
*/

    //Defining the subscriber variables
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer2_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher2_;


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
    
    size_t count_;

};





void endeffectorValues(const std::shared_ptr<rob_kinematics_server::srv::Velreference::Request> request,
          std::shared_ptr<rob_kinematics_server::srv::Velreference::Response>      response)
{

  arma::colvec joint_vel(3);
  for (int i = 0; i < 3; ++i)
   {
     joint_vel(i) = request->joint_velocity[i];
   } 

  

  Jacobian.rows(0,2).col(0) = arma::cross(zv.col(0),ov.col(0));
  Jacobian.rows(0,2).col(1) = arma::cross(zv.col(1),ov.col(1));
  Jacobian.rows(0,2).col(2) = zv.col(2);
  Jacobian.rows(3,5).col(0) = zv.col(0);
  Jacobian.rows(3,5).col(1) = zv.col(1);

  ov.print("ov: ");
  zv.print("zv: ");
  Jacobian.print("Jaocbian: ");


  // Jacobian.print("Jacobian: ");
  // zv.print("z: ");

  arma::vec end_vel(6);


  for(int i =0;i<6;i++){
    end_vel(i) = Jacobian(i,0)*joint_vel(0) + Jacobian(i,1)*joint_vel(1) + Jacobian(i,2)*joint_vel(2) ;
  }


  for(int i =0; i< 6; i++){
    response->end_effect_vel[i] = end_vel(i);
  }

   std::cout<<response->end_effect_vel[0]<<std::endl;

   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nx: %f" " y: %f" " z: %f",
                 joint_vel(0), joint_vel(1), joint_vel(2));
   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%0.2f, %0.2f, %0.2f]", end_vel(0), end_vel(1), end_vel(2));

}




void jointValues(const std::shared_ptr<rob_kinematics_server::srv::Jointvel::Request> request,
          std::shared_ptr<rob_kinematics_server::srv::Jointvel::Response>      response)
{

  
  for (int i = 0; i < 6; ++i)
   {
     end_effector_velocities(i) = request->end_effector_vel[i];
   } 



  Jacobian.rows(0,2).col(0) = arma::cross(zv.col(0),ov.col(0));
  Jacobian.rows(0,2).col(1) = arma::cross(zv.col(1),ov.col(1));
  Jacobian.rows(0,2).col(2) = zv.col(2);
  Jacobian.rows(3,5).col(0) = zv.col(0);
  Jacobian.rows(3,5).col(1) = zv.col(1);

  arma::mat Jacobian_inverse = arma::pinv(Jacobian); 


  for(int i =0;i<3;i++){
    desired_joint_velocities(i) = Jacobian_inverse(i,0)*end_effector_velocities(0) + Jacobian_inverse(i,1)*end_effector_velocities(1) + Jacobian_inverse(i,2)*end_effector_velocities(2) + Jacobian_inverse(i,3)*end_effector_velocities(3) + Jacobian_inverse(i,4)*end_effector_velocities(4)+ Jacobian_inverse(i,5)*end_effector_velocities(5) ;
  }


  for(int i =0; i< 3; i++){
    response->joint_vel[i] = desired_joint_velocities(i);
  }

  ref_received = true;


   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nx: %f" " y: %f" " z: %f",
                 end_effector_velocities(0), end_effector_velocities(1), end_effector_velocities(2));
   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%0.2f, %0.2f, %0.2f]", desired_joint_velocities(0), desired_joint_velocities(1), desired_joint_velocities(2));

}



int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("end_effector_velocity_server");

  std::shared_ptr<rclcpp::Node> node2 = rclcpp::Node::make_shared("joint_velocity_server");

  rclcpp::Service<rob_kinematics_server::srv::Velreference>::SharedPtr service =
  node->create_service<rob_kinematics_server::srv::Velreference>("end_effector_values_service", endeffectorValues);

  rclcpp::Service<rob_kinematics_server::srv::Jointvel>::SharedPtr service2 =
  node2->create_service<rob_kinematics_server::srv::Jointvel>("joint_values_service", jointValues);


  auto node1 = std::make_shared<InputjointSubscriberforvelocity>();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to send joint values.");

  exec.add_node(node);
  exec.add_node(node1);
  exec.add_node(node2);
  exec.spin();

  rclcpp::shutdown();

  return 0;

}
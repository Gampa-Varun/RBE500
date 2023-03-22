#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/quaternion.hpp" 
#include "geometry_msgs/msg/pose.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define _USE_MATH_DEFINES
#include <cmath>



//Importing required libraries

using std::placeholders::_1;
using namespace std::chrono_literals;

// Defining the subscriber class which contains the set of subscribers required for kinematics
class InputjointSubscriber : public rclcpp::Node
{
  public:
    InputjointSubscriber()
    : Node("kinematics"), count_(0)
    {
    	
      // Biniding the subscriber call to callback functions (forward and inverse for forward and inverse kinematics)
      // Setting the pointer to subscriber calls

      //Subscribing to input_joint_angles for forward kinematics
      subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "forward_position_controller/commands", 10, std::bind(&InputjointSubscriber::forward_callback, this, _1));


      publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/SCARA/end_effector_pose", 10); 
      timer_ = this->create_wall_timer(
      1000ms, std::bind(&InputjointSubscriber::timer_callback, this));
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
    void forward_callback(const std_msgs::msg::Float64MultiArray & msg) 
    {
      //Obtaining the theta values from the Float32MultiArray msg
      float rz, ry,rz_2;
      theta[0] = msg.data[0];
      theta[1] = msg.data[1];
      d[2] = msg.data[2]+1;
     
      std::vector<std::vector<float>> Ai(4,std::vector<float>(4));
      std::vector<std::vector<float>> H(4,std::vector<float>(4)); 
      H = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
      
      
  
      //Creating Ai matrices and multiplying them to the Homogenous matrices
      for (int i = 0; i < 3; ++i)
      {
        Ai = DH_matrix(theta[i],alpha[i],a[i],d[i]);
        H = multiply_small(H,Ai);
      }
      

      //Outputing the matrix
       for(int i = 0; i < 4; i++)
      {
        for(int j = 0; j < 4; j++)
        {
          if(H[i][j]<0){
          printf("%0.2f ", H[i][j] );
        }
        else{
          printf(" %0.2f ", H[i][j]);
        }
        }
        std::cout << std::endl;
      } 

      for(int i = 0; i<3;i++){
      	position[i] = H[i][3];
      }

      //Calculating the euler angles from the homogenous matrix
      rz_2 = atan2(H[2][1],-H[2][0]);
      rz = atan2(H[1][2],H[0][2]);
      ry = atan2(H[0][2]*cos(rz)+ H[1][2]*sin(rz),H[2][2]);

      printf("Ez   %f \n",rz*180/M_PI );
      printf("Ey   %f \n",ry*180/M_PI);
      printf ("Ez_2 %f \n",rz_2*180/M_PI);

      //<Note that if needed we can get quaternions from the euler to quaternion node using this>

      std::cout << "----------------"<<std::endl;      
    }


    void timer_callback()
    {
      auto message = std_msgs::msg::Float64MultiArray();
      message.data.push_back(position[0]);
      message.data.push_back(position[1]);
      message.data.push_back(position[2]);
      //RCLCPP_INFO(this->get_logger(), "Publishing...");
      publisher_->publish(message);
    }

    //Defining the subscriber valraibles
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;

    //Defining the system parameters
    float a[3] = {1,1,0};
    float alpha[3] = {0,3.14,0};
    float d[3] = {2,0,1};
    float theta[3] = {0,0,0};
    float position[3] = {2,0,1};
    
    size_t count_;

  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InputjointSubscriber>());
  rclcpp::shutdown();
  return 0;
}

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/quaternion.hpp" 
#include "geometry_msgs/msg/pose.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define _USE_MATH_DEFINES
#include <cmath>


using std::placeholders::_1;
using namespace std::chrono_literals;

class InputjointSubscriber : public rclcpp::Node
{
  public:
    InputjointSubscriber()
    : Node("kinematics")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "input_joint_values", 10, std::bind(&InputjointSubscriber::forward_callback, this, _1));

      subscription_pose = this->create_subscription<geometry_msgs::msg::Pose>(
      "end_effector_pose", 10, std::bind(&InputjointSubscriber::inverse_callback, this, _1)); 
    }

  private:

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

    std::vector<std::vector<float>> DH_matrix( float theta , float alpha, float a, float d  ){
        std::vector<std::vector<float>> Ai(4,std::vector<float>(4));
        Ai= {{cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)}, {sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)}, {0, sin(alpha), cos(alpha), d}, {0, 0, 0, 1}};
        return Ai;
    }    



    void forward_callback(const std_msgs::msg::Float32MultiArray & msg) 
    {
      float rz, ry,rz_2;
      theta[0] = msg.data[0]*M_PI/180;
      theta[1] = msg.data[1]*M_PI/180;
      theta[2] = msg.data[2]*M_PI/180;
     
      std::vector<std::vector<float>> Ai(4,std::vector<float>(4));
      std::vector<std::vector<float>> H(4,std::vector<float>(4));

      H = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
  
      for (int i = 0; i < 3; ++i)
      {
        Ai = DH_matrix(theta[i],alpha[i],a[i],d[i]);
        H = multiply_small(H,Ai);
      }
      

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

      rz_2 = atan2(H[2][1],-H[2][0]);
      rz = atan2(H[1][2],H[0][2]);
      ry = atan2(H[0][2]*cos(rz)+ H[1][2]*sin(rz),H[2][2]);

      printf("Ez   %f \n",rz*180/M_PI );
      printf("Ey   %f \n",ry*180/M_PI);
      printf ("Ez_2 %f \n",rz_2*180/M_PI);


      std::cout << "---------------"<<std::endl;      
    }


    void inverse_callback(const geometry_msgs::msg::Pose & msg){
      float x_c,y_c,z_c,qx_c,qy_c,qz_c,qw_c;
      float theta[3];
      x_c = msg.position.x;
      y_c = msg.position.y;
      z_c = msg.position.z;
      qx_c = msg.orientation.x;
      qy_c = msg.orientation.y;
      qz_c = msg.orientation.z;
      qw_c = msg.orientation.w;

      theta[0] = atan2(y_c,x_c);


      float r = sqrt(x_c*x_c + y_c*y_c);  
      float s = d[0] - z_c;
      float D = (x_c*x_c + y_c*y_c + (z_c-d[0])*(z_c-d[0]) - a[1]*a[1] - a[2]*a[2])/(2*a[1]*a[2]);

      theta[2] =  atan2(sqrt(1-D*D),D);

      theta[1] = atan2(s,r) - atan2(a[2]*sin(theta[2]),a[1]+a[2]*cos(theta[2]));

      for (int i = 0; i < 3; ++i)
      {
        printf("%f \n", theta[i]*180/M_PI);
      }

    }


    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_pose;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    float a[3] = {0,1,1};
    float alpha[3] = {-90*M_PI/180,0,0};
    float d[3] = {1,0,0};
    float theta[3] = {0,0,0};

  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InputjointSubscriber>());
  rclcpp::shutdown();
  return 0;
}

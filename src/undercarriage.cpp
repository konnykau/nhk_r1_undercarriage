#include <sensor_msgs/msg/joy.hpp>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nhk_r1_undercarriage/undercarriage.hpp"
#include "robomas_package_2/msg/motor_cmd_array.hpp"
#include "robomas_package_2/msg/motor_cmd.hpp"
#include "nhk_r1_undercarriage/msg/velocity_vector.hpp"

using std::placeholders::_1;

constexpr double parallel_velocity = 1.0; //m/s todo
constexpr double rotation_velocity = 0.4; //rad/s todo
constexpr double machine_radius = 0.28; //m todo


class Undercarriage_Node: public rclcpp::Node
{
public:
  Undercarriage_Node()
  : Node("nhk_r1_undercarriage")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&Undercarriage_Node::joy_callback, this, _1));
    auto_moving_mode_subscription_ = this->create_subscription<nhk_r1_undercarriage::msg::VelocityVector>(
      "auto_moving_mode_velocity_vector", 10, std::bind(&Undercarriage_Node::auto_moving_mode_callback, this, _1));
    robomas_pub_ = this->create_publisher<robomas_package_2::msg::MotorCmdArray>("motor_cmd_array", 10);

  }
  

private:
  void joy_callback(const sensor_msgs::msg::Joy & msg)
  {  
    if(msg.buttons[7]){//startボタン
      nhk_r1_undercarriage_.make_mode(motor_mode::velocity);
      
    }//mode velにする
    if(msg.buttons[6]){//backボタン
      nhk_r1_undercarriage_.make_mode(motor_mode::disable);
    }//mode disにする
    if(msg.buttons[9]){//左スティック押し込みボタン
      auto_moving_mode_ = !auto_moving_mode_;
      if(auto_moving_mode_){
        RCLCPP_INFO(this->get_logger(),"auto moving mode ON");
      }
      else{
        RCLCPP_INFO(this->get_logger(),"auto moving mode OFF");
      }
    }
///////////////////////////////ここの上がstartボタン、backボタンによるmodeの調整
///////////////////////////////ここの下から平行移動、回転をするための個々のモーターのターゲットを決めるif文

   
    if(auto_moving_mode_){
      return;
    }
    float x = msg.axes[0] * parallel_velocity;//x方向への平行移動速度
    float y = msg.axes[1] * parallel_velocity;//y方向への平行移動速度
    float theta_vel = 0.0; //回転速度
    if(msg.axes[2] == -1){//ZL(left shouldderボタン)
      theta_vel = -rotation_velocity * machine_radius;
    }//left turn
    else if(msg.axes[5] == -1){//ZR(right shouldderボタン)
      theta_vel = -rotation_velocity * machine_radius; 
    }//right turn//平行移動//x軸はjoyの入力時点で反転していた
    this->nhk_r1_undercarriage_.update(x,y,theta_vel);

    robomas_pub_->publish(this->nhk_r1_undercarriage_.make_robomas_Frame());
  }

    void auto_moving_mode_callback(const nhk_r1_undercarriage::msg::VelocityVector & msg)
    {
      if(!auto_moving_mode_){
        return;
      }
      RCLCPP_INFO(this->get_logger(),"auto moving mode velocity vector received: x:%f, y:%f, theta:%f",msg.x,msg.y,msg.theta);
    }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::Subscription<nhk_r1_undercarriage::msg::VelocityVector>::SharedPtr auto_moving_mode_subscription_;
  rclcpp::Publisher<robomas_package_2::msg::MotorCmdArray>::SharedPtr robomas_pub_;
  undercarriage nhk_r1_undercarriage_;
  bool auto_moving_mode_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Undercarriage_Node>());
  rclcpp::shutdown();
  return 0;
}


// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.



 #include <chrono>
 #include <cmath>
 #include <limits>
 #include <memory>
 #include <vector>
 #include <string>

// //#include "rclcpp/rclcpp.hpp"
 #include "rclcpp/rclcpp.hpp"
 #include "std_msgs/msg/int32.hpp"
 #include "std_msgs/msg/string.hpp"

//#include "hardware_interface/types/hardware_interface_type_values.hpp"

//#include "diffbot_system.hpp"
//#include "diffbot_node.hpp"



//using diffbot_base::diffbot_node;
using namespace std::chrono_literals;
using std::placeholders::_1;




class EncoderLeftSubscriber : public rclcpp::Node
{
  public:
    EncoderLeftSubscriber()
    : Node("diffbot_encoder_left")
    {
      encoder_left_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "encoder_left", 10, std::bind(&EncoderLeftSubscriber::encoder_left_receive_callback, this, _1));
    }

  private:
    void encoder_left_receive_callback(const std_msgs::msg::Int32::SharedPtr encleft) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard left encoder : '%d'", encleft->data );
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr encoder_left_subscription_;
};



class EncoderRightSubscriber : public rclcpp::Node
{
  public:
    EncoderRightSubscriber()
    : Node("diffbot_encoder_right")
    {
      encoder_right_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "encoder_right", 10, std::bind(&EncoderRightSubscriber::encoder_right_receive_callback, this, _1));
    }

  private:
    void encoder_right_receive_callback(const std_msgs::msg::Int32::SharedPtr encright) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard right encoder : '%d'", encright->data );
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr encoder_right_subscription_;
};







int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);  
  rclcpp::executors::StaticSingleThreadedExecutor executor;

  auto encoder_left_node = std::make_shared<EncoderLeftSubscriber>();
  auto encoder_right_node = std::make_shared<EncoderRightSubscriber>(); 

  executor.add_node(encoder_left_node);
  executor.add_node(encoder_right_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}












































/*

diffbot_node::diffbot_node()
: Node("diffbot_subenc")
{
    //RCLCPP_INFO(get_logger(), "diffbot_node Node Init");

    //motor_left_rpm_publisher_  = this->create_publisher<std_msgs::msg::Int32>("motor_left_rpm", 1);
    //motor_right_rpm_publisher_  = this->create_publisher<std_msgs::msg::Int32>("motor_right_rpm", 1);    

    //timer_ = this->create_wall_timer(
    //100ms , std::bind( &diffbot_node::timer_callback, this ) );


  encoder_left_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
    "encoder_left",
    10,
    std::bind(&diffbot_node::encoder_left_rx_callback),
    this,
    _1
    );


}
*/




/*

void diffbot_node::encoder_left_rx_callback()
{  
    RCLCPP_INFO(
      rclcpp::get_logger("MicroRosNode"),
      "Got left encoder pulse  %d   !", 555
      );    
}

*/



/*

void diffbot_node::timer_callback()
{  

  auto motor_left_rpm = std_msgs::msg::Int32();
  motor_left_rpm.data = 360;
  motor_left_rpm_publisher_->publish(motor_left_rpm);

  auto motor_right_rpm = std_msgs::msg::Int32();
  motor_right_rpm.data = 360;
  motor_right_rpm_publisher_->publish(motor_right_rpm);

  RCLCPP_INFO(this->get_logger(), "Publishing left motor rpm: '%d'", motor_left_rpm.data );  
  RCLCPP_INFO(this->get_logger(), "Publishing right motor rpm: '%d'", motor_right_rpm.data );    
  RCLCPP_INFO(this->get_logger(), "Publishing right motor rpm: '%d'", motor_right_rpm.data );    
    RCLCPP_INFO(this->get_logger(), "Publishing right motor rpm: '%d'", motor_right_rpm.data );    
      RCLCPP_INFO(this->get_logger(), "Publishing right motor rpm: '%d'", motor_right_rpm.data );      
}
*/





/*
int main(int argc, char* argv[])
{
  //init
  rclcpp::init(argc,argv);

  //excutor define
  std::shared_ptr<rclcpp::Executor> executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor();

  // Run the node(s)
    executor->add_node("diffbot_node");

    rclcpp::spin(std::make_shared<diffbot_node>());

    // Exit
    rclcpp::shutdown();

    return 0;
}

*/
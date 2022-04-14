
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
// #include <cmath>
// #include <limits>
// #include <memory>
// #include <vector>
// #include <string>

// //#include "rclcpp/rclcpp.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/int32.hpp"
// #include "std_msgs/msg/string.hpp"

//#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "diffbot_node.hpp"



using diffbot_base::diffbot_node;
using namespace std::chrono_literals;



diffbot_node::diffbot_node()
 : Node("diffbot_node")
{
    RCLCPP_INFO(get_logger(), "diffbot_node Node Init");

    motor_left_rpm_publisher_  = this->create_publisher<std_msgs::msg::Int32>("motor_left_rpm", 1);
    motor_right_rpm_publisher_  = this->create_publisher<std_msgs::msg::Int32>("motor_right_rpm", 1);    


    timer_ = this->create_wall_timer(
      100ms , std::bind( &diffbot_node::timer_callback, this ) );
}




void diffbot_node::timer_callback()
{  

  auto motor_left_rpm = std_msgs::msg::Int32();
  motor_left_rpm.data = 360;
  motor_left_rpm_publisher_->publish(motor_left_rpm);

  auto motor_right_rpm = std_msgs::msg::Int32();
  motor_right_rpm.data = 360;
  motor_right_rpm_publisher_->publish(motor_right_rpm);

  //RCLCPP_INFO(this->get_logger(), "Publishing left motor rpm: '%d'", motor_left_rpm.data );  
  //RCLCPP_INFO(this->get_logger(), "Publishing right motor rpm: '%d'", motor_right_rpm.data );    
}







int main(int argc, char* argv[])
{
  //init
  rclcpp::init(argc,argv);

  //excutor define
  //std::shared_ptr<rclcpp::Executor> executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor();

  // Run the node(s)
    //executor->add_node(diffbot_base);

    rclcpp::spin(std::make_shared<diffbot_node>());

    // Exit
    rclcpp::shutdown();

    return 0;
}


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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"



#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "diffbot_msg/srv/encoderservice.hpp"



#include "diffbot_system.hpp"
//#include "diffbot_node.hpp"



using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;



namespace diffbot_hardware
{


void get_values_client(
  int32_t& encoder_left_val,
  int32_t& encoder_right_val,
  rclcpp::Time& encoder_left_timestamp,
  rclcpp::Time& encoder_right_timestamp )
 {
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("lgv_encoders_hw_interface");
    rclcpp::Client<diffbot_msg::srv::Encoderservice>::SharedPtr client =
    node->create_client<diffbot_msg::srv::Encoderservice>("encoder_val");

   auto request = std::make_shared<diffbot_msg::srv::Encoderservice::Request>();
    request->state =true;

    while (!client->wait_for_service(1s)) 
    {
      if (!rclcpp::ok()) 
      {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting."); break;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    auto result = client->async_send_request(request);

    // Wait for the result.
   if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
      //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "left: %0.5f,right: %0.5f ", result.get()->to_encoder_left,result.get()->to_encoder_right);
      encoder_left_val = result.get()->to_encoder_left;
      encoder_right_val = result.get()->to_encoder_right;

      //rclcpp::Time get_time_temp1 = result.get()->to_encoder_left_stamp;
      //rclcpp::Time get_time_temp2 = result.get()->to_encoder_right_stamp;
      encoder_left_timestamp = result.get()->to_encoder_left_stamp;
      encoder_right_timestamp = result.get()->to_encoder_right_stamp;


     //RCLCPP_INFO(rclcpp::get_logger("Debug_from_hardware_node"), "lft time stamp: %5ld. val: %d, right time stamp: %5ld, val: %d ", 
     //             get_time_temp1.nanoseconds(), result.get()->to_encoder_left, get_time_temp2.nanoseconds(), result.get()->to_encoder_right);

    } 
    else 
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service encoder");
    }
  }
















HardwareCommandPub::HardwareCommandPub() : Node("hardware_command_publisher")
{
  motor_left_rpm_publisher_ = this->create_publisher<std_msgs::msg::Int32>("motor_left_rpm", 1);
  motor_right_rpm_publisher_ = this->create_publisher<std_msgs::msg::Int32>("motor_right_rpm", 1);
}


void HardwareCommandPub::MotorLeftRpmPublish(std_msgs::msg::Int32 motor_left_rpm)
{
  motor_left_rpm_publisher_->publish(motor_left_rpm);
}

void HardwareCommandPub::MotorRightRpmPublish(std_msgs::msg::Int32 motor_right_rpm)
{
  motor_right_rpm_publisher_->publish(motor_right_rpm);
}



/*
HardwareCommandSub::HardwareCommandSub() : Node("hardware_command_subscriber")
{
  encoder_left_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
    "encoder_left",
    1,
    std::bind(&HardwareCommandSub::encoder_left_callback, this, 1)
    );
}


void encoder_left_callback(const std_msgs::msg::Int32 encoder_left)
{
    RCLCPP_INFO(
      rclcpp::get_logger("MicroRosNode"),
      "Got left encoder pulse  %d   !", encoder_left.data
      );
}

*/






CallbackReturn DiffBotSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  base_x_ = 0.0;
  base_y_ = 0.0;
  base_theta_ = 0.0;





  hw_cmd_motor_left_rpm_pub_ = std::make_shared<HardwareCommandPub>();  //fire up the publisher node
  hw_cmd_motor_right_rpm_pub_ = std::make_shared<HardwareCommandPub>();  //fire up the publisher node





  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // START: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }














  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

CallbackReturn DiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // START: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Activating ...please wait...");

  for (auto i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("DiffBotSystemHardware"), "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully activated!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn DiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // START: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Deactivating ...please wait...");

  for (auto i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("DiffBotSystemHardware"), "%.1f seconds left...", hw_stop_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully deactivated!");

  return CallbackReturn::SUCCESS;
}












//0.628318400 


#define DIFFBOT_WHEEL_RADIUS 0.285
#define ENCODER_MAX  112572

//( PI * 휠지름[0.2m])/(1회전엔코더[112572] * 기어비[1]) 
#define METER_PER_ENCODERTICK 0.00000558148029705433f //meter


// RPM = (given M/S / ( PI * 2 ) ) * 60
//#define VELOCITY_CONVERT_CONSTANT 0.318309952
#define VELOCITY_CONVERT_CONSTANT 19.09859714















hardware_interface::return_type DiffBotSystemHardware::read()
{

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Reading...");

  double radius = 0.1;  // radius of the wheels
  double dist_w = 0.57;   // distance between the wheels

  double ldt = 0.0;      // Control period
  double rdt = 0.0;      // Control period

  //odom calculate from diffbot encoder pulse
  //wheel differentiable kinematics

  //get encoder current value
  get_values_client(CurrentEncoderPulseLeft, CurrentEncoderPulseRight, CurrentEncoderPulseLeftTimestamp, CurrentEncoderPulseRightTimestamp);



  RCLCPP_INFO( rclcpp::get_logger("diffbot_odom"), "Got value from thread: %.5f, %.5f .",hw_positions_[0],hw_positions_[1]);  
  RCLCPP_INFO( rclcpp::get_logger("diffbot_odom"), "Got value from diffbot_node: %d, %d .",CurrentEncoderPulseLeft, CurrentEncoderPulseRight);


  //cal duration
  ldt   =    CurrentEncoderPulseLeftTimestamp.seconds() - LastEncoderPulseLeftTimestamp.seconds() ;
  rdt   =    CurrentEncoderPulseRightTimestamp.seconds() - LastEncoderPulseRightTimestamp.seconds() ;  




  RCLCPP_INFO( rclcpp::get_logger("diffbot_odom"), "Duration: %.5f, %.5f .",ldt, rdt);    


 
  //cal diff encoder value
  DiffEncoderPulseLeft = CurrentEncoderPulseLeft - LastEncoderPulseLeft;
  if( DiffEncoderPulseLeft < 0           ) DiffEncoderPulseLeft = DiffEncoderPulseLeft + ENCODER_MAX;
  if( DiffEncoderPulseLeft > ENCODER_MAX ) DiffEncoderPulseLeft = DiffEncoderPulseLeft - ENCODER_MAX;

  DiffEncoderPulseRight = CurrentEncoderPulseRight - LastEncoderPulseRight;
  if( DiffEncoderPulseRight < 0           ) DiffEncoderPulseRight = DiffEncoderPulseRight + ENCODER_MAX;
  if( DiffEncoderPulseRight > ENCODER_MAX ) DiffEncoderPulseRight = DiffEncoderPulseRight - ENCODER_MAX;

  RCLCPP_INFO( rclcpp::get_logger("diffbot_odom"), "diff_left: %d, diff_right: %d .",DiffEncoderPulseLeft, DiffEncoderPulseRight);



  //need max encoder limit cal
  //update last encoder value
  LastEncoderPulseLeft = CurrentEncoderPulseLeft;
  LastEncoderPulseRight = CurrentEncoderPulseRight;

  //time value bacup
  LastEncoderPulseLeftTimestamp = CurrentEncoderPulseLeftTimestamp;
  LastEncoderPulseRightTimestamp = CurrentEncoderPulseRightTimestamp;

  

  //calculate left and right wheel travel distance
  DistanceTravelledLeftWheel = DiffEncoderPulseLeft*METER_PER_ENCODERTICK;
  DistanceTravelledRightWheel = DiffEncoderPulseLeft*METER_PER_ENCODERTICK;
  RCLCPP_INFO( rclcpp::get_logger("diffbot_odom"), "DistanceTravelledLeftWheel: %.5f, DistanceTravelledRightWheel: %.5f .",DistanceTravelledLeftWheel, DistanceTravelledRightWheel);


  //double base_dx = 0.5 * radius * (hw_commands_[0] + hw_commands_[1]) * cos(base_theta_);
  double base_dx = 0.5 * radius * (DistanceTravelledLeftWheel + DistanceTravelledRightWheel) * cos(base_theta_);

  //double base_dy = 0.5 * radius * (hw_commands_[0] + hw_commands_[1]) * sin(base_theta_);  
  double base_dy = 0.5 * radius * (DistanceTravelledLeftWheel - DistanceTravelledRightWheel) * cos(base_theta_);

  //double base_dtheta = radius * (hw_commands_[0] - hw_commands_[1]) / dist_w;
  double base_dtheta = radius * (DistanceTravelledLeftWheel - DistanceTravelledRightWheel) / dist_w;


  //normalize angle
  base_dtheta = fmod(base_dtheta, 2.0*M_PI);
  if (base_dtheta < 0) base_dtheta += 2.0*M_PI;


  //update joint state - pos
  hw_positions_[0] = hw_positions_[0] + DistanceTravelledLeftWheel;
  hw_positions_[1] = hw_positions_[1] + DistanceTravelledRightWheel;

  //update joint state - vel
  hw_velocities_[0] = DistanceTravelledLeftWheel/ldt;
  hw_velocities_[1] = DistanceTravelledRightWheel/rdt;  


  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    // START: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(
      rclcpp::get_logger("DiffBotSystemHardware"),
      "Got position state %.5f and velocity state %.5f for '%s'!", hw_positions_[i],
      hw_velocities_[i], info_.joints[i].name.c_str());
    // END: This part here is for exemplary purposes - Please do not copy to your production code
  }


  //update diffbot x,y,theta
  base_x_ += base_dx * ldt;
  base_y_ += base_dy * rdt;
  base_theta_ += base_dtheta * ldt;

  RCLCPP_INFO( rclcpp::get_logger("DiffBotSystemHardware"), "base_x_:%.5f, base_y_:%.5f, base_theta_:%.5f. ",base_x_,base_y_,base_theta_);






/*

  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    // Simulate DiffBot wheels's movement as a first-order system
    // Update the joint status: this is a revolute joint without any limit.
    // Simply integrates
    hw_positions_[i] = hw_positions_[1] + dt * hw_commands_[i];
    hw_velocities_[i] = hw_commands_[i];

    // START: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(
      rclcpp::get_logger("DiffBotSystemHardware"),
      "Got position state %.5f and velocity state %.5f for '%s'!", hw_positions_[i],
      hw_velocities_[i], info_.joints[i].name.c_str());
    // END: This part here is for exemplary purposes - Please do not copy to your production code
  }
*/

  // Update the free-flyer, i.e. the base notation using the classical
  // wheel differentiable kinematics
  /*
  double base_dx = 0.5 * radius * (hw_commands_[0] + hw_commands_[1]) * cos(base_theta_);
  double base_dy = 0.5 * radius * (hw_commands_[0] + hw_commands_[1]) * sin(base_theta_);
  double base_dtheta = radius * (hw_commands_[0] - hw_commands_[1]) / dist_w;
  base_x_ += base_dx * dt;
  base_y_ += base_dy * dt;
  base_theta_ += base_dtheta * dt;
*/





  // START: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("DiffBotSystemHardware"), "Joints successfully read! (%.5f,%.5f,%.5f)",
    base_x_, base_y_, base_theta_);
  // END: This part here is for exemplary purposes - Please do not copy to your production code



  return hardware_interface::return_type::OK;
}









hardware_interface::return_type diffbot_hardware::DiffBotSystemHardware::write()
{
  // START: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Writing...");

  for (auto i = 0u; i < hw_commands_.size(); i++)
  {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("DiffBotSystemHardware"), "Got command %.5f for '%s'!", hw_commands_[i],
      info_.joints[i].name.c_str());
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Joints successfully written!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code


  // sending commands to the hardware
  //publish to topic
  motor_left_rpm_.data = (int32_t)(hw_commands_[0] * VELOCITY_CONVERT_CONSTANT);
  hw_cmd_motor_left_rpm_pub_->MotorLeftRpmPublish(motor_left_rpm_);  

  RCLCPP_INFO(
    rclcpp::get_logger("MiroRosNode"), "pub left wheel command %d for '%s'!", motor_left_rpm_.data,
    info_.joints[0].name.c_str());


  //publish to topic
  motor_right_rpm_.data = (int32_t)(hw_commands_[1] * VELOCITY_CONVERT_CONSTANT);
  hw_cmd_motor_right_rpm_pub_->MotorRightRpmPublish(motor_right_rpm_);  

  RCLCPP_INFO(
    rclcpp::get_logger("MiroRosNode"), "pub right wheel command %d for '%s'!", motor_right_rpm_.data,
    info_.joints[1].name.c_str());

  return hardware_interface::return_type::OK;
}




}  // namespace diffbot_hardware



#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffbot_hardware::DiffBotSystemHardware, hardware_interface::SystemInterface)

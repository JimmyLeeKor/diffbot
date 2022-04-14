
#ifndef __DIFFBOT_BASE_HPP__
#define __DIFFBOT_BASE_HPP__


#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"



namespace diffbot_base
{

  class diffbot_node
   : public rclcpp::Node
  {
    public:
      explicit diffbot_node();
      virtual ~diffbot_node() {}

      //int32_t get_motor_left_rpm();
      //void set_motor_left_rpm(int32_t rpm);
      //rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motor_left_rpm_publisher_;

      void set_motor_left_rpm(int32_t lw_rpm_);
      void set_motor_right_rpm(int32_t rw_rpm_);


    private:

      rclcpp::TimerBase::SharedPtr timer_;
      void timer_callback();
      rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motor_left_rpm_publisher_;
      rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motor_right_rpm_publisher_;      
      
      size_t count_;

      int32_t lw_rpm_;
      int32_t rw_rpm_;




      //void motor_left_rpm_publisher();

      
      //int32_t motor_left_rpm_;
      //int32_t motor_right_rpm;


  };
} //namespace








#endif // __DIFFBOT_BASE_HPP__
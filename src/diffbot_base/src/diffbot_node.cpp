#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
//#include "std_msgs/msg/float64.hpp"

#include "diffbot_msg/srv/encoderservice.hpp"
#include "std_srvs/srv/empty.hpp"



//for std::bind
using std::placeholders::_1;
using std::placeholders::_2;

class encoder : public rclcpp::Node
{
  public:
    encoder(): Node("lgv_hardware_read_buffer_node")
    {
        encoder_left_subs = this->create_subscription<std_msgs::msg::Int32>("encoder_left", 1, std::bind(&encoder::get_encoder_left, this, _1));
        encoder_right_subs = this->create_subscription<std_msgs::msg::Int32>("encoder_right", 1, std::bind(&encoder::get_encoder_right, this, _1));
        send_interface = this->create_service<diffbot_msg::srv::Encoderservice>("encoder_val", std::bind(&encoder::send_to_interface, 
        this,_1,_2));
    }

    private:
      void get_encoder_left(const std_msgs::msg::Int32 & msg)
      {

        encoder_left_buffer = encoder_left_buffer + msg.data;
        encoder_accum_counter++;

        //RCLCPP_INFO(this->get_logger(), "msg.data: '%.5f'   leftw travell acc: '%.5f'  accumm cnt '%.1f'", msg.data, encoder_left_buffer, encoder_accum_counter);        
      }

      void get_encoder_right(const std_msgs::msg::Int32 & msg)
      {
        encoder_right_buffer =  encoder_right_buffer + msg.data;

        //RCLCPP_INFO(this->get_logger(), "right wheel travelled distance accumm: '%.5f'", encoder_right_buffer);        
      }

    public:
      void send_to_interface(const std::shared_ptr<diffbot_msg::srv::Encoderservice::Request>  request
       ,std::shared_ptr<diffbot_msg::srv::Encoderservice::Response>  response)
      {
            if(request->state)
            {
                response->to_encoder_left = encoder_left_buffer;
                encoder_left_buffer = 0;

                //response->to_encoder_left_stamp = encoder_left_stamp_buffer;
                response->to_encoder_right = encoder_right_buffer;
                encoder_right_buffer = 0;

                response->to_encoder_accumulate_count = encoder_accum_counter;
                encoder_accum_counter = 0 ;
                //response->to_encoder_right_stamp = encoder_right_stamp_buffer;
               // RCLCPP_INFO(rclcpp::get_logger("Debug_from_buffer_node"), "left time stamp: %5ld. val: %d,right time stamp: %5ld, val: %d ", 
               //   encoder_left_stamp_buffer, encoder_left_buffer, encoder_right_stamp_buffer, encoder_right_buffer);
                //RCLCPP_INFO(this->get_logger(), "send service response");
            }
      }

    private:
      int32_t encoder_left_buffer;
      int32_t encoder_right_buffer;
      int32_t encoder_accum_counter;      


      rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr encoder_left_subs;
      rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr encoder_right_subs;
      rclcpp::Service<diffbot_msg::srv::Encoderservice>::SharedPtr send_interface;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto test_node = std::make_shared<encoder>();
  rclcpp::spin(test_node);
  rclcpp::shutdown();
  return 0;
}
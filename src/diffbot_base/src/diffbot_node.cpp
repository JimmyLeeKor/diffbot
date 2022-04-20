#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"

#include "diffbot_base/srv/encoderservice.hpp"
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
        send_interface = this->create_service<diffbot_base::srv::Encoderservice>("encoder_val", std::bind(&encoder::send_to_interface, 
        this,_1,_2));
    }

    private:
      void get_encoder_left(const std_msgs::msg::Int32 & msg)
      {
        //RCLCPP_INFO(this->get_logger(), "encoder_left: '%d'", msg.data);
        std_msgs::msg::Int32 temp = msg;
        encoder_left_buffer = (int32_t) temp.data;
      }

      void get_encoder_right(const std_msgs::msg::Int32 & msg)
      {
        //RCLCPP_INFO(this->get_logger(), "encoder_right: '%d'", msg.data);
        std_msgs::msg::Int32 temp = msg;
        encoder_right_buffer = (int32_t) temp.data;
      }

    public:
      void send_to_interface(const std::shared_ptr<diffbot_base::srv::Encoderservice::Request>  request
       ,std::shared_ptr<diffbot_base::srv::Encoderservice::Response>  response)
      {
            if(request->state)
            {
                response->to_encoder_left = (double)encoder_left_buffer;
                response->to_encoder_right = (double)encoder_right_buffer;
                //RCLCPP_INFO(this->get_logger(), "send service response");
            }
      }

    private:
      int32_t encoder_left_buffer;
      int32_t encoder_right_buffer;

      rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr encoder_left_subs;
      rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr encoder_right_subs;
      rclcpp::Service<diffbot_base::srv::Encoderservice>::SharedPtr send_interface;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto test_node = std::make_shared<encoder>();
  rclcpp::spin(test_node);
  rclcpp::shutdown();
  return 0;
}
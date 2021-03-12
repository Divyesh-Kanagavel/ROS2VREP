#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
using namespace std::chrono_literals;

class SpeedPublisher : public rclcpp::Node
{
public:
    SpeedPublisher() : Node("pioneer_pub")
    {
    leftpublisher_ = this->create_publisher<std_msgs::msg::Int32>("leftMotorSpeed",10);
    rightpublisher_ = this->create_publisher<std_msgs::msg::Int32>("rightMotorSpeed",10);
    timer_ = this->create_wall_timer(500ms, std::bind(&SpeedPublisher::timer_callback, this));
    }
    
    private:
    void timer_callback()
    {
    auto message_left = std_msgs::msg::Int32();
    message_left.data = 5;
    RCLCPP_INFO(this->get_logger(), "Publishing left wheel speed: '%d'", message_left.data);
    leftpublisher_->publish(message_left);
    
    auto message_right = std_msgs::msg::Int32();
    message_right.data = 5;
    RCLCPP_INFO(this->get_logger(), "Publishing right wheel speed: '%d'",message_right.data);
    rightpublisher_->publish(message_right);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr leftpublisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr rightpublisher_;
    
    };
    
    int main(int argc, char* argv[])
    {
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<SpeedPublisher>());
    rclcpp::shutdown();
    return 0;
    }

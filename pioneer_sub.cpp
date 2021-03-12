#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
using std::placeholders::_1;

class ForceSubscriber : public rclcpp::Node
{
  public:
    ForceSubscriber()
    : Node("pioneer_sub")
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "forcevalue", 10, std::bind(&ForceSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "The x,y,z component of force is %f,%f,%f", msg->x,msg->y, msg->z );
    }
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ForceSubscriber>());
  rclcpp::shutdown();
  return 0;
}

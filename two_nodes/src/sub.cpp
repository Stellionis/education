#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


static constexpr char* c_topic = "publisher";
static const uint8_t c_queue_size = 10;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      _psubscription = this->create_subscription<std_msgs::msg::String>(
      c_topic, c_queue_size, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
    }

  private:
    void topic_callback(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
  private:  
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _psubscription;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}


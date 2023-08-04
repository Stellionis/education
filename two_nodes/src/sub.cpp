#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int16.hpp"

static constexpr char * c_topic = "publisher";
static constexpr char * c_subtopic = "subscriber";
static const uint8_t c_queue_size = 10;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("subscriber")
  {
    _psubscription = this->create_subscription<std_msgs::msg::String>(
      c_topic, c_queue_size,
      std::bind(&MinimalSubscriber::topicCallback, this, std::placeholders::_1));
    _ppublisher = this->create_publisher<std_msgs::msg::Int16>(c_subtopic, c_queue_size);
  }

private:
  void topicCallback(const std_msgs::msg::String & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

    auto message = std_msgs::msg::Int16();
    message.data = 12345;
    RCLCPP_INFO(this->get_logger(), std::to_string(message.data).c_str());
    _ppublisher->publish(message);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _psubscription;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr _ppublisher;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

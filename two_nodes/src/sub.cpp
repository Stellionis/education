#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int16.hpp"

static constexpr char * c_pub_topic = "publisher";
static constexpr char * c_sub_topic = "subscriber";

static constexpr char * c_queue_size_param = "queue_size";
static const uint8_t c_queue_size = 10;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("subscriber")
  {
    this->declare_parameter(c_queue_size_param, c_queue_size);
    uint8_t queue_size = this->get_parameter(c_queue_size_param).as_int();

    _psubscription = this->create_subscription<std_msgs::msg::String>(
      c_pub_topic, queue_size,
      std::bind(&MinimalSubscriber::topicCallback, this, std::placeholders::_1));
    _ppublisher = this->create_publisher<std_msgs::msg::Int16>(c_sub_topic, queue_size);
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

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int16.hpp"

static constexpr char* c_topic = "publisher";
static constexpr char* c_subtopic = "subscriber";
static const uint8_t c_queue_size = 10;
static const uint8_t c_time_interval_sec = 2;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("publisher"), _count(0)
    {
      _ppublisher = this->create_publisher<std_msgs::msg::String>(c_topic, c_queue_size);
      _psubscription = this->create_subscription<std_msgs::msg::Int16>(
      c_subtopic, c_queue_size, std::bind(&MinimalPublisher::сallback, this, std::placeholders::_1));
      _ptimer = this->create_wall_timer(
      std::chrono::seconds(c_time_interval_sec), std::bind(&MinimalPublisher::timerCallback, this));
    }

  private:
    void timerCallback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, I'm alive! "+ std::to_string(_count++);
      RCLCPP_INFO(this->get_logger(), message.data.c_str());
      _ppublisher->publish(message);
    }

  private:
    void сallback(const std_msgs::msg::Int16 & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", std::to_string(msg.data).c_str());     
    }

  private:  
    rclcpp::TimerBase::SharedPtr _ptimer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _ppublisher;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr _psubscription;
    size_t _count;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int16.hpp"

static constexpr char * c_pub_topic = "publisher";
static constexpr char * c_sub_topic = "subscriber";

static constexpr char * c_time_interval_param_sec = "timer_polling_frequency";
static const uint8_t с_timer_interval_param_value = 2;

static constexpr char * c_queue_size_param = "queue_size";
static const uint8_t c_queue_size = 10;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("publisher"), _count(0)
  {
    this->declare_parameter(c_time_interval_param_sec, с_timer_interval_param_value);
    uint8_t timer_frequency = this->get_parameter(c_time_interval_param_sec).as_int();

    this->declare_parameter(c_queue_size_param, c_queue_size);
    uint8_t queue_size = this->get_parameter(c_queue_size_param).as_int();

    _ppublisher = this->create_publisher<std_msgs::msg::String>(c_pub_topic, queue_size);
    _psubscription = this->create_subscription<std_msgs::msg::Int16>(
      c_sub_topic, queue_size,
      std::bind(&MinimalPublisher::сallback, this, std::placeholders::_1));
    _ptimer = this->create_wall_timer(
      std::chrono::seconds(timer_frequency), std::bind(&MinimalPublisher::timerCallback, this));
  }

private:
  void timerCallback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, I'm alive! " + std::to_string(_count++);
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

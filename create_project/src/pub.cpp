#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("publisher"), _count(0)
    {
      const uint16_t c_time_interval_sec = 2;
      using namespace std::chrono_literals;
      _publisher = this->create_publisher<std_msgs::msg::String>("publisher", 10);
      timer = this->create_wall_timer(
      c_time_interval_sec * 1s, std::bind(&MinimalPublisher::timerCallback, this));
    }

  private:
    void timerCallback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, I'm alive!" + std::to_string(_count++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      _publisher->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher;
    size_t _count;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

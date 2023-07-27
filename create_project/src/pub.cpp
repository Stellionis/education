#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

static constexpr char* c_topic = "publisher";
static const uint8_t c_queue_size = 10;
static const uint8_t _time_interval_sec = 2;
class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("publisher"), _count(0)
    {
      _ppublisher = this->create_publisher<std_msgs::msg::String>(c_topic, c_queue_size);
      _ptimer = this->create_wall_timer(
      std::chrono::seconds(_time_interval_sec), std::bind(&MinimalPublisher::timerCallback, this));
    }

  private:

    void timerCallback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, I'm alive!" + std::to_string(_count++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      _ppublisher->publish(message);
    }
    rclcpp::TimerBase::SharedPtr _ptimer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _ppublisher;
    size_t _count;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

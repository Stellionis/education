#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

static constexpr char* c_topic = "publisher";
static const uint8_t c_queue_size = 10;
static const uint8_t c_time_interval_sec = 2;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), _pcount(0)
    {
      _ppublisher_ = this->create_publisher<std_msgs::msg::String>(c_topic, c_queue_size);
      _ptimer_ = this->create_wall_timer(
      std::chrono::seconds(c_time_interval_sec), std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, I'm alive! " + std::to_string(_pcount++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      _ppublisher_->publish(message);
    }
  private:  
    rclcpp::TimerBase::SharedPtr _ptimer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _ppublisher_;
    size_t _pcount;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

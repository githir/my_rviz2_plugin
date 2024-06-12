#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <random>
#include <chrono>

class SignalGenerator : public rclcpp::Node
{
public:
  SignalGenerator() : Node("signal_generator")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("/signals", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&SignalGenerator::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = generate_random_signal();
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  std::string generate_random_signal()
  {
    std::string signal(12, '0');
    for (char &c : signal)
    {
      c = (rand() % 2) ? '1' : '0';
    }
    return signal;
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SignalGenerator>());
  rclcpp::shutdown();
  return 0;
}
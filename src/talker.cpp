#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using namespace std::chrono_literals;

class Talker : public rclcpp::Node
{
public:
  Talker()
  : Node("talker")
  {
    // Declare parameters v (speed) and d (steering angle)
    this->declare_parameter("v", 1.0);
    this->declare_parameter("d", 0.0);

    // Create a publisher for AckermannDriveStamped messages
    publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

    // Timer to publish as fast as possible
    timer_ = this->create_wall_timer(10ms, std::bind(&Talker::publish_drive_message, this));
  }

private:
  void publish_drive_message()
  {
    // Get the parameter values for speed (v) and steering angle (d)
    double v = this->get_parameter("v").as_double();
    double d = this->get_parameter("d").as_double();

    // Create and populate the AckermannDriveStamped message
    auto message = ackermann_msgs::msg::AckermannDriveStamped();
    message.drive.speed = v;
    message.drive.steering_angle = d;

    // Log and publish the message
    RCLCPP_INFO(this->get_logger(), "Publishing: speed=%.2f, steering_angle=%.2f", v, d);
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}

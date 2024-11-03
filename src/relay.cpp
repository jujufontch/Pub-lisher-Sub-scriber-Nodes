#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class Relay : public rclcpp::Node
{
public:
  Relay()
  : Node("relay")
  {
    // Create a subscriber to the "drive" topic
    subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "drive", 10, std::bind(&Relay::drive_callback, this, std::placeholders::_1));

    // Create a publisher for the modified message on "drive_relay" topic
    publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive_relay", 10);
  }

private:
  void drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
  {
    // Modify speed and steering_angle by multiplying them by 3
    auto modified_msg = ackermann_msgs::msg::AckermannDriveStamped();
    modified_msg.drive.speed = msg->drive.speed * 3;
    modified_msg.drive.steering_angle = msg->drive.steering_angle * 3;

    // Log and publish the modified message
    RCLCPP_INFO(this->get_logger(), "Relaying: speed=%.2f, steering_angle=%.2f",
                modified_msg.drive.speed, modified_msg.drive.steering_angle);
    publisher_->publish(modified_msg);
  }

  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscription_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Relay>());
  rclcpp::shutdown();
  return 0;
}

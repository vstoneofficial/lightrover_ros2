#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class GamePadNode : public rclcpp::Node
{
public:
  GamePadNode()
  : Node("rover_gamepad"), debug_counter_(0)
  {
    this->declare_parameter<double>("linear_scale", 0.3);
    this->declare_parameter<double>("angular_scale", 2.0);

    linear_scale_ = this->get_parameter("linear_scale").as_double();
    angular_scale_ = this->get_parameter("angular_scale").as_double();

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("rover_twist", 1);
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 1,
      std::bind(&GamePadNode::on_joy, this, std::placeholders::_1));
  }

private:
  void on_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    geometry_msgs::msg::Twist speed;
    if (msg->axes.size() > 2) {
      speed.linear.x = msg->axes[1] * linear_scale_;
      speed.angular.z = msg->axes[2] * angular_scale_;
    }

    ++debug_counter_;
    if (debug_counter_ % 20 == 0) {
      RCLCPP_INFO(
        this->get_logger(),
        "Joy -> rover_twist linear.x=%.3f angular.z=%.3f",
        speed.linear.x, speed.angular.z);
    }

    publisher_->publish(speed);
  }

  double linear_scale_;
  double angular_scale_;
  int debug_counter_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GamePadNode>();
  RCLCPP_INFO(node->get_logger(), "Game pad node start");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

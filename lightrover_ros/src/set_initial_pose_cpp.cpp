#include <cmath>
#include <memory>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/srv/set_initial_pose.hpp"
#include "rclcpp/rclcpp.hpp"

class InitialPoseSetterNode : public rclcpp::Node
{
public:
  InitialPoseSetterNode()
  : Node("initial_pose_setter")
  {
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<std::string>("topic_name", "/initialpose");
    this->declare_parameter<std::string>("service_name", "/set_initial_pose");
    this->declare_parameter<double>("x", 0.0);
    this->declare_parameter<double>("y", 0.0);
    this->declare_parameter<double>("yaw", 0.0);
    this->declare_parameter<double>("wait_sec", 5.0);
    this->declare_parameter<double>("retry_sec", 2.0);
    this->declare_parameter<int>("max_attempts", 3);
    this->declare_parameter<bool>("use_current_stamp", false);
    this->declare_parameter<double>("covariance_x", 0.25);
    this->declare_parameter<double>("covariance_y", 0.25);
    this->declare_parameter<double>("covariance_yaw", 0.06853891945200942);

    start_time_ = this->now();
    last_attempt_time_ = start_time_;

    publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      this->get_parameter("topic_name").as_string(), 10);
    amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", 10, std::bind(&InitialPoseSetterNode::on_amcl_pose, this, std::placeholders::_1));
    client_ = this->create_client<nav2_msgs::srv::SetInitialPose>(
      this->get_parameter("service_name").as_string());
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&InitialPoseSetterNode::tick, this));
  }

private:
  geometry_msgs::msg::PoseWithCovarianceStamped build_pose() const
  {
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    if (this->get_parameter("use_current_stamp").as_bool()) {
      pose.header.stamp = this->now();
    }
    pose.header.frame_id = this->get_parameter("frame_id").as_string();
    pose.pose.pose.position.x = this->get_parameter("x").as_double();
    pose.pose.pose.position.y = this->get_parameter("y").as_double();

    const double yaw = this->get_parameter("yaw").as_double();
    pose.pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.pose.orientation.w = std::cos(yaw / 2.0);

    pose.pose.covariance[0] = this->get_parameter("covariance_x").as_double();
    pose.pose.covariance[7] = this->get_parameter("covariance_y").as_double();
    pose.pose.covariance[35] = this->get_parameter("covariance_yaw").as_double();
    return pose;
  }

  void on_amcl_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr)
  {
    if (amcl_pose_received_) {
      return;
    }

    amcl_pose_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Received /amcl_pose, initial localization is active");
    finish();
  }

  void tick()
  {
    if (amcl_pose_received_) {
      return;
    }

    const auto now = this->now();
    const double elapsed = (now - start_time_).seconds();
    if (elapsed < this->get_parameter("wait_sec").as_double()) {
      return;
    }

    const double retry_sec = this->get_parameter("retry_sec").as_double();
    if (attempt_count_ > 0 && (now - last_attempt_time_).seconds() < retry_sec) {
      return;
    }

    if (attempt_count_ >= this->get_parameter("max_attempts").as_int()) {
      RCLCPP_ERROR(this->get_logger(), "AMCL did not publish /amcl_pose after repeated initial pose attempts");
      finish();
      return;
    }

    auto pose = build_pose();
    publisher_->publish(pose);
    ++attempt_count_;
    last_attempt_time_ = now;

    RCLCPP_INFO(
      this->get_logger(), "Published /initialpose attempt %d x=%.3f y=%.3f yaw=%.3f",
      attempt_count_, pose.pose.pose.position.x, pose.pose.pose.position.y,
      this->get_parameter("yaw").as_double());

    if (!client_->wait_for_service(std::chrono::milliseconds(200))) {
      RCLCPP_WARN(this->get_logger(), "Waiting for /set_initial_pose service...");
      return;
    }

    auto request = std::make_shared<nav2_msgs::srv::SetInitialPose::Request>();
    request->pose = pose;
    client_->async_send_request(
      request,
      [this](rclcpp::Client<nav2_msgs::srv::SetInitialPose>::SharedFuture future) {
        try {
          (void)future.get();
          RCLCPP_INFO(this->get_logger(), "Initial pose sent through /set_initial_pose");
        } catch (const std::exception & e) {
          RCLCPP_ERROR(this->get_logger(), "Failed to call /set_initial_pose: %s", e.what());
        }
      });
  }

  void finish()
  {
    timer_.reset();
    done_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this]() {
      RCLCPP_INFO(this->get_logger(), "Initial pose helper finished");
      rclcpp::shutdown();
    });
  }

  bool amcl_pose_received_{false};
  int attempt_count_{0};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_attempt_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Client<nav2_msgs::srv::SetInitialPose>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr done_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InitialPoseSetterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

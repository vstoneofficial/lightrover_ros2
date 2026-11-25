#include <cmath>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class NavGoalSenderNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavGoalSenderNode()
  : Node("nav_goal_sender")
  {
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<double>("x", 0.5);
    this->declare_parameter<double>("y", 0.0);
    this->declare_parameter<double>("yaw", 0.0);
    this->declare_parameter<std::string>("server_name", "/navigate_to_pose");
    this->declare_parameter<std::string>("cmd_vel_topic", "/rover_twist");
    this->declare_parameter<double>("server_wait_sec", 30.0);
    this->declare_parameter<double>("result_wait_sec", 40.0);

    start_time_ = this->now();

    action_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, this->get_parameter("server_name").as_string());
    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      this->get_parameter("cmd_vel_topic").as_string(), 10,
      std::bind(&NavGoalSenderNode::on_cmd_vel, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&NavGoalSenderNode::tick, this));
  }

private:
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (first_twist_logged_) {
      return;
    }
    first_twist_logged_ = true;
    RCLCPP_INFO(
      this->get_logger(), "Received rover_twist linear.x=%.3f angular.z=%.3f",
      msg->linear.x, msg->angular.z);
  }

  void tick()
  {
    const double elapsed = (this->now() - start_time_).seconds();

    if (!goal_sent_) {
      if (elapsed > this->get_parameter("server_wait_sec").as_double()) {
        RCLCPP_ERROR(this->get_logger(), "Timed out waiting for NavigateToPose action server");
        finish();
        return;
      }

      if (!action_client_->wait_for_action_server(std::chrono::milliseconds(100))) {
        return;
      }

      send_goal();
      return;
    }

    if (result_received_) {
      finish();
      return;
    }

    if (elapsed > this->get_parameter("result_wait_sec").as_double()) {
      RCLCPP_WARN(this->get_logger(), "Timed out waiting for navigation result");
      finish();
    }
  }

  void send_goal()
  {
    NavigateToPose::Goal goal;
    goal.pose = geometry_msgs::msg::PoseStamped();
    goal.pose.header.frame_id = this->get_parameter("frame_id").as_string();
    goal.pose.header.stamp = this->now();
    goal.pose.pose.position.x = this->get_parameter("x").as_double();
    goal.pose.pose.position.y = this->get_parameter("y").as_double();

    const double yaw = this->get_parameter("yaw").as_double();
    goal.pose.pose.orientation.z = std::sin(yaw / 2.0);
    goal.pose.pose.orientation.w = std::cos(yaw / 2.0);

    goal_sent_ = true;
    RCLCPP_INFO(
      this->get_logger(), "Sending NavigateToPose goal x=%.3f y=%.3f yaw=%.3f",
      goal.pose.pose.position.x, goal.pose.pose.position.y, yaw);

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.feedback_callback =
      [this](GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        const double nav_time = static_cast<double>(feedback->navigation_time.sec) +
          static_cast<double>(feedback->navigation_time.nanosec) / 1e9;
        RCLCPP_INFO(
          this->get_logger(), "Feedback distance_remaining=%.3f navigation_time=%.3f",
          feedback->distance_remaining, nav_time);
      };
    options.goal_response_callback = [this](const GoalHandleNavigateToPose::SharedPtr & handle) {
        if (!handle) {
          RCLCPP_ERROR(this->get_logger(), "NavigateToPose goal was rejected");
          result_received_ = true;
          return;
        }
        goal_handle_ = handle;
        RCLCPP_INFO(this->get_logger(), "NavigateToPose goal accepted");
      };
    options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult & result) {
        RCLCPP_INFO(this->get_logger(), "NavigateToPose finished with status=%d", static_cast<int>(result.code));
        result_received_ = true;
      };

    action_client_->async_send_goal(goal, options);
  }

  void finish()
  {
    timer_.reset();
    done_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this]() {
      RCLCPP_INFO(this->get_logger(), "Navigation goal helper finished");
      rclcpp::shutdown();
    });
  }

  bool goal_sent_{false};
  bool result_received_{false};
  bool first_twist_logged_{false};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};

  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  GoalHandleNavigateToPose::SharedPtr goal_handle_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr done_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavGoalSenderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

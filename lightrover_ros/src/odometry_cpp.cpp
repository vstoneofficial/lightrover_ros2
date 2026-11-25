#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <future>
#include <memory>
#include <optional>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "lightrover_interface/srv/wrc201_msg.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace
{
constexpr uint8_t MS32_M_POS0 = 0x60;
constexpr int DIFF_COUNT_LIMIT = 1048575;
constexpr int STARTUP_STABLE_COUNT_DELTA = 128;
constexpr int STARTUP_STABLE_SAMPLE_COUNT = 5;

constexpr double WHEEL_CIRCUMFERENCE = 60.0 * M_PI / 1000.0;
constexpr double ENC_COUNTS_PER_TURN = 1188.024;
constexpr double ENC_PER_M = ENC_COUNTS_PER_TURN / WHEEL_CIRCUMFERENCE;

constexpr double ROVER_D = 0.143 / 2.0;
constexpr double MAX_LINEAR_SPEED_MPS = 0.6;
constexpr double MAX_ANGULAR_SPEED_RADPS = 4.0;
}

class OdometryNode : public rclcpp::Node
{
public:
  OdometryNode()
  : Node("wrc201_odometry"), odom_br_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    client_ = this->create_client<lightrover_interface::srv::Wrc201Msg>("wrc201_i2c");
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    last_stamp_ = this->now();

    timer_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&OdometryNode::update_odom, this));
  }

private:
  static std::array<int32_t, 2> unpack_two_signed32(int64_t value)
  {
    std::array<int32_t, 2> out{};
    std::memcpy(out.data(), &value, 8);
    return out;
  }

  rclcpp::Client<lightrover_interface::srv::Wrc201Msg>::SharedFuture make_request(int32_t addr, int32_t length, const std::string & cmd)
  {
    auto req = std::make_shared<lightrover_interface::srv::Wrc201Msg::Request>();
    req->addr = addr;
    req->data = 0;
    req->length = length;
    req->cmd = cmd;
    return client_->async_send_request(req).future.share();
  }

  geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw) const
  {
    geometry_msgs::msg::Quaternion q;
    q.z = std::sin(yaw / 2.0);
    q.w = std::cos(yaw / 2.0);
    return q;
  }

  bool cal_speed(const std::array<int32_t, 2> & enc_val, const rclcpp::Time & current_stamp, double & linear_x, double & angular_z, double & dt)
  {
    if (!encoder_initialized_) {
      if (!init_candidate_.has_value()) {
        init_candidate_ = enc_val;
        init_stable_count_ = 1;
        last_stamp_ = current_stamp;
        linear_x = 0.0;
        angular_z = 0.0;
        dt = 0.0;
        return true;
      }

      if (std::abs(enc_val[0] - (*init_candidate_)[0]) > STARTUP_STABLE_COUNT_DELTA ||
          std::abs(enc_val[1] - (*init_candidate_)[1]) > STARTUP_STABLE_COUNT_DELTA)
      {
        init_candidate_ = enc_val;
        init_stable_count_ = 1;
        last_stamp_ = current_stamp;
        linear_x = 0.0;
        angular_z = 0.0;
        dt = 0.0;
        return true;
      }

      ++init_stable_count_;
      if (init_stable_count_ < STARTUP_STABLE_SAMPLE_COUNT) {
        last_stamp_ = current_stamp;
        linear_x = 0.0;
        angular_z = 0.0;
        dt = 0.0;
        return true;
      }

      pre_count_ = {static_cast<double>(enc_val[0]), static_cast<double>(enc_val[1])};
      last_stamp_ = current_stamp;
      encoder_initialized_ = true;
      init_candidate_.reset();
      init_stable_count_ = 0;
      linear_x = 0.0;
      angular_z = 0.0;
      dt = 0.0;
      return true;
    }

    dt = (current_stamp - last_stamp_).seconds();
    last_stamp_ = current_stamp;
    if (dt <= 0.0) {
      return false;
    }

    for (size_t i = 0; i < 2; ++i) {
      const double diff = static_cast<double>(enc_val[i]) - pre_count_[i];
      if (std::abs(diff) < DIFF_COUNT_LIMIT) {
        diff_count_[i] = -1.0 * diff;
      }
    }
    pre_count_ = {static_cast<double>(enc_val[0]), static_cast<double>(enc_val[1])};

    const std::array<double, 2> distance{{diff_count_[0] / ENC_PER_M, diff_count_[1] / ENC_PER_M}};
    const std::array<double, 2> speed{{distance[0] / dt, distance[1] / dt}};

    linear_x = (speed[0] - speed[1]) / 2.0;
    angular_z = -1.0 * ((speed[0] + speed[1]) / (2.0 * ROVER_D));

    if (std::abs(linear_x) > MAX_LINEAR_SPEED_MPS || std::abs(angular_z) > MAX_ANGULAR_SPEED_RADPS) {
      pre_count_ = {static_cast<double>(enc_val[0]), static_cast<double>(enc_val[1])};
      diff_count_ = {0.0, 0.0};
      RCLCPP_WARN(this->get_logger(), "Discarding implausible odom sample linear_x=%.4f angular_z=%.4f", linear_x, angular_z);
      linear_x = 0.0;
      angular_z = 0.0;
      return true;
    }

    return true;
  }

  void cal_odometry(double vx, double vth, double dt)
  {
    x_ += vx * std::cos(th_) * dt;
    y_ += vx * std::sin(th_) * dt;
    th_ += vth * dt;
  }

  void publish_odom(double linear_x, double angular_z, const rclcpp::Time & stamp)
  {
    auto odom_quat = yaw_to_quaternion(th_);

    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = stamp;
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_footprint";
    odom_tf.transform.translation.x = x_;
    odom_tf.transform.translation.y = y_;
    odom_tf.transform.rotation = odom_quat;
    odom_br_->sendTransform(odom_tf);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation = odom_quat;
    odom.twist.twist.linear.x = linear_x;
    odom.twist.twist.angular.z = angular_z;
    odom_pub_->publish(odom);
  }

  void update_odom()
  {
    const auto current_stamp = this->now();
    double linear_x = 0.0;
    double angular_z = 0.0;

    if (!client_->wait_for_service(std::chrono::milliseconds(10))) {
      if (debug_counter_ % 50 == 0) {
        RCLCPP_WARN(this->get_logger(), "Encoder service is not available");
      }
      ++debug_counter_;
      publish_odom(linear_x, angular_z, current_stamp);
      return;
    }

    if (!enc_future_.valid()) {
      enc_future_ = make_request(MS32_M_POS0, 8, "enc_pair_r");
      publish_odom(linear_x, angular_z, current_stamp);
      return;
    }

    if (enc_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      publish_odom(linear_x, angular_z, current_stamp);
      return;
    }

    auto res = enc_future_.get();
    enc_future_ = {};

    if (!res) {
      if (debug_counter_ % 50 == 0) {
        RCLCPP_WARN(this->get_logger(), "Encoder future completed without a response");
      }
      ++debug_counter_;
      enc_future_ = make_request(MS32_M_POS0, 8, "enc_pair_r");
      publish_odom(linear_x, angular_z, current_stamp);
      return;
    }

    auto enc = unpack_two_signed32(res->read_data);

    if (debug_counter_ % 20 == 0) {
      RCLCPP_INFO(this->get_logger(), "Encoder raw values A=%d B=%d", enc[0], enc[1]);
    }

    double dt = 0.0;
    const bool ok = cal_speed(enc, current_stamp, linear_x, angular_z, dt);
    if (!ok) {
      if (debug_counter_ % 50 == 0) {
        RCLCPP_WARN(this->get_logger(), "Skipping odom update because dt <= 0");
      }
      ++debug_counter_;
      enc_future_ = make_request(MS32_M_POS0, 8, "enc_pair_r");
      publish_odom(linear_x, angular_z, current_stamp);
      return;
    }

    if (debug_counter_ % 20 == 0) {
      RCLCPP_INFO(
        this->get_logger(), "Odom update linear_x=%.4f angular_z=%.4f x=%.4f y=%.4f th=%.4f",
        linear_x, angular_z, x_, y_, th_);
    }
    ++debug_counter_;

    cal_odometry(linear_x, angular_z, dt);
    publish_odom(linear_x, angular_z, current_stamp);

    enc_future_ = make_request(MS32_M_POS0, 8, "enc_pair_r");
  }

  rclcpp::Client<lightrover_interface::srv::Wrc201Msg>::SharedPtr client_;
  rclcpp::Client<lightrover_interface::srv::Wrc201Msg>::SharedFuture enc_future_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_br_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::array<double, 2> pre_count_{{0.0, 0.0}};
  std::array<double, 2> diff_count_{{0.0, 0.0}};
  bool encoder_initialized_{false};
  std::optional<std::array<int32_t, 2>> init_candidate_;
  int init_stable_count_{0};

  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  double x_{0.0};
  double y_{0.0};
  double th_{0.0};
  int debug_counter_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdometryNode>();
  RCLCPP_INFO(node->get_logger(), "Start odom manager");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

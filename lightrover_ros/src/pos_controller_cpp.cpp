#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <future>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "lightrover_interface/srv/wrc201_msg.hpp"
#include "lightrover_ros/vs_wrc201_motor.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr uint8_t MU8_O_EN = 0x10;
constexpr uint8_t MU8_TRIG = 0x11;
constexpr uint8_t MS16_FB_PG0 = 0x20;
constexpr uint8_t MS16_FB_PG1 = 0x22;
constexpr uint8_t MS32_A_POS0 = 0x48;
constexpr uint8_t MU16_FB_PCH0 = 0x30;
constexpr uint8_t MU16_FB_PCH1 = 0x32;
constexpr double ROVER_D = 0.143 / 2.0;
}

class PosControllerNode : public rclcpp::Node
{
public:
  PosControllerNode()
  : Node("pos_controller")
  {
    // 速度指令のタイムアウト [s]
    // 最後の指令からこの時間を超えて新しい指令が来なければ目標速度を0にする
    // 0以下を指定するとタイムアウトを無効にする
    cmd_timeout_ = this->declare_parameter<double>("cmd_timeout", 0.5);

    client_ = this->create_client<lightrover_interface::srv::Wrc201Msg>("wrc201_i2c");

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&PosControllerNode::on_odom, this, std::placeholders::_1));
    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "rover_twist", 10, std::bind(&PosControllerNode::on_target, this, std::placeholders::_1));
    nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&PosControllerNode::on_target, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(30),
      std::bind(&PosControllerNode::control_loop, this));
  }

  void initialize_motor()
  {
    send_async(MU8_O_EN, 0x00, 1, "w");
    send_async(MU8_TRIG, 0x0c, 1, "w");
    send_async(MS16_FB_PG0, 0x0080, 2, "w");
    send_async(MS16_FB_PG1, 0x0080, 2, "w");
    send_async(MU16_FB_PCH0, 0x09C4, 2, "w");
    send_async(MU16_FB_PCH1, 0x09C4, 2, "w");
    send_async(MU8_O_EN, 0x03, 1, "w");
  }

  bool wait_for_backend_service(std::chrono::seconds timeout)
  {
    return client_->wait_for_service(timeout);
  }

private:
  static int64_t pack_two_signed32(int32_t low_value, int32_t high_value)
  {
    uint8_t bytes[8];
    std::memcpy(bytes, &low_value, 4);
    std::memcpy(bytes + 4, &high_value, 4);
    int64_t packed = 0;
    std::memcpy(&packed, bytes, 8);
    return packed;
  }

  lightrover_interface::srv::Wrc201Msg::Request::SharedPtr make_req(
    int32_t addr, int64_t data, int32_t length, const std::string & cmd)
  {
    auto req = std::make_shared<lightrover_interface::srv::Wrc201Msg::Request>();
    req->addr = addr;
    req->data = data;
    req->length = length;
    req->cmd = cmd;
    return req;
  }

  rclcpp::Client<lightrover_interface::srv::Wrc201Msg>::SharedFuture send_async(
    int32_t addr, int64_t data, int32_t length, const std::string & cmd)
  {
    return client_->async_send_request(make_req(addr, data, length, cmd)).future.share();
  }

  bool has_pending_write() const
  {
    return pending_write_.valid() &&
           pending_write_.wait_for(std::chrono::seconds(0)) != std::future_status::ready;
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const double linear_x = msg->twist.twist.linear.x;
    const double angular_z = msg->twist.twist.angular.z;

    current_v_[1] = linear_x + ROVER_D * angular_z;
    current_v_[0] = -1.0 * (linear_x - ROVER_D * angular_z);
  }

  void on_target(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    target_rover_v_[1] = msg->linear.x + ROVER_D * msg->angular.z;
    target_rover_v_[0] = -1.0 * (msg->linear.x - ROVER_D * msg->angular.z);

    last_cmd_time_ = std::chrono::steady_clock::now();
    has_cmd_ = true;
    if (cmd_timed_out_) {
      RCLCPP_INFO(this->get_logger(), "Velocity command received again, resuming");
      cmd_timed_out_ = false;
    }
  }

  void check_cmd_timeout()
  {
    if (cmd_timeout_ <= 0.0 || !has_cmd_ || cmd_timed_out_) {
      return;
    }
    const std::chrono::duration<double> elapsed =
      std::chrono::steady_clock::now() - last_cmd_time_;
    if (elapsed.count() > cmd_timeout_) {
      // 停止指令(0)を受けた後に指令が途絶えるのは正常なので警告しない
      if (target_rover_v_[0] != 0.0 || target_rover_v_[1] != 0.0) {
        RCLCPP_WARN(
          this->get_logger(), "No velocity command for %.2f s, stopping the rover", cmd_timeout_);
      }
      target_rover_v_[0] = 0.0;
      target_rover_v_[1] = 0.0;
      cmd_timed_out_ = true;
    }
  }

  void control_loop()
  {
    check_cmd_timeout();

    // 前回の書き込みが終わっていなければ、移動量の計算ごと次の周期に回す。
    // pos_control() は前回呼び出しからの経過時間ぶんの移動量を返すため、
    // 計算した後で送信をスキップすると、その分の移動量が失われて
    // CPU 負荷が高いとき (SLAM 実行中など) に指令より遅く・カクついて走る。
    if (has_pending_write()) {
      ++skipped_writes_;
      if (debug_counter_ % 20 == 0) {
        RCLCPP_WARN(
          this->get_logger(),
          "Motor write is still pending, deferring (%d skipped so far)", skipped_writes_);
      }
      ++debug_counter_;
      return;
    }

    const double now_sec = this->now().seconds();
    const auto output = motor_controller_.pos_control(current_v_, target_rover_v_, now_sec);

    if (debug_counter_ % 20 == 0) {
      RCLCPP_INFO(
        this->get_logger(),
        "Control current_v=(%.3f, %.3f) target_v=(%.3f, %.3f) output=(%d, %d)",
        current_v_[0], current_v_[1], target_rover_v_[0], target_rover_v_[1], output[0], output[1]);
    }

    ++debug_counter_;
    const int64_t packed_output = pack_two_signed32(output[0], output[1]);
    pending_write_ = send_async(MS32_A_POS0, packed_output, 8, "motor_w");
  }

  int debug_counter_{0};
  int skipped_writes_{0};
  double cmd_timeout_{0.5};
  std::chrono::steady_clock::time_point last_cmd_time_;
  bool has_cmd_{false};
  bool cmd_timed_out_{false};
  std::array<double, 2> current_v_{{0.0, 0.0}};
  std::array<double, 2> target_rover_v_{{0.0, 0.0}};

  lightrover_ros::VsWrc201Motor motor_controller_;

  rclcpp::Client<lightrover_interface::srv::Wrc201Msg>::SharedPtr client_;
  rclcpp::Client<lightrover_interface::srv::Wrc201Msg>::SharedFuture pending_write_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PosControllerNode>();

  while (rclcpp::ok() && !node->wait_for_backend_service(std::chrono::seconds(1)))
  {
    RCLCPP_INFO(node->get_logger(), "Service is not available");
  }

  node->initialize_motor();
  RCLCPP_INFO(node->get_logger(), "Start POS Controll");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

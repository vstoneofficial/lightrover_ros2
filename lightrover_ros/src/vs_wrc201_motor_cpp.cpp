#include "lightrover_ros/vs_wrc201_motor.hpp"

namespace lightrover_ros
{

void VsWrc201Motor::control_to_velocity(double now_sec)
{
  std::array<double, 2> v_diff{{ctl_v_com_[0] - v_com_[0], ctl_v_com_[1] - v_com_[1]}};
  const double max_v_diff = std::max(std::abs(v_diff[0]), std::abs(v_diff[1]));

  if (pre_t_calc_2_v_ < 0.0) {
    pre_t_calc_2_v_ = now_sec;
  }

  const double elapsed_time = now_sec - pre_t_calc_2_v_;

  if (max_v_diff > 0.0) {
    for (size_t i = 0; i < 2; ++i) {
      const double add_v = motor_param_[8] * (v_diff[i] / max_v_diff) * elapsed_time;
      if (std::abs(v_diff[i]) >= std::abs(add_v)) {
        v_com_[i] += add_v;
      } else {
        v_com_[i] = ctl_v_com_[i];
      }
    }
  }

  for (size_t i = 0; i < 2; ++i) {
    if (ctl_v_com_[i] == 0.0 && std::abs(v_com_[i]) <= 0.02) {
      v_com_[i] = 0.0;
    }
  }

  pre_t_calc_2_v_ = now_sec;
}

std::array<int32_t, 2> VsWrc201Motor::pos_control(
  const std::array<double, 2> & /*rover_v*/,
  const std::array<double, 2> & target_v,
  double now_sec)
{
  ctl_v_com_ = target_v;

  control_to_velocity(now_sec);

  const std::array<double, 2> e_v_com{{v_com_[0] * ENC_PER_M, v_com_[1] * ENC_PER_M}};

  if (pre_t_pos_ctl_ < 0.0) {
    pre_t_pos_ctl_ = now_sec;
  }
  const double elapsed_time = now_sec - pre_t_pos_ctl_;
  pre_t_pos_ctl_ = now_sec;

  buf_enc_com_[0] += e_v_com[0] * elapsed_time;
  buf_enc_com_[1] += e_v_com[1] * elapsed_time;

  std::array<int32_t, 2> enc_com{{
    static_cast<int32_t>(buf_enc_com_[0]),
    static_cast<int32_t>(buf_enc_com_[1])
  }};

  buf_enc_com_[0] -= enc_com[0];
  buf_enc_com_[1] -= enc_com[1];

  return enc_com;
}

}  // namespace lightrover_ros

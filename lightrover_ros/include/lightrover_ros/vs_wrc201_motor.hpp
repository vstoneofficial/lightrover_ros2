#pragma once

#include <array>
#include <cmath>
#include <cstdint>

namespace lightrover_ros
{

class VsWrc201Motor
{
public:
  std::array<int32_t, 2> pos_control(
    const std::array<double, 2> & rover_v,
    const std::array<double, 2> & target_v,
    double now_sec);

private:
  void control_to_velocity(double now_sec);

  static constexpr double ROVER_D = 0.0717767;
  static constexpr double TIRE_CIRCUMFERENCE = 60.0 * M_PI / 1000.0;
  static constexpr double ENC_COUNTS_PER_TURN = 1188.024;
  static constexpr double ENC_PER_M = ENC_COUNTS_PER_TURN / TIRE_CIRCUMFERENCE;

  std::array<double, 9> std_motor_param_{{0.5, M_PI, 520.0, 3.6, 2.0, 3.6, 30.0, 127.0, 1.5}};
  std::array<double, 9> motor_param_{{0.5, M_PI, 520.0, 3.6, 2.0, 3.6, 30.0, 127.0, 1.5}};

  std::array<double, 2> ctl_v_com_{{0.0, 0.0}};
  std::array<double, 2> v_com_{{0.0, 0.0}};
  std::array<double, 2> buf_enc_com_{{0.0, 0.0}};

  double pre_t_calc_2_v_{-1.0};
  double pre_t_pos_ctl_{-1.0};
};

}  // namespace lightrover_ros

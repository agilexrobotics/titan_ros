/**
 * @file titan_params.hpp
 * @date 2021-04-20
 * @brief 
 * 
 # @copyright Copyright (c) 2024 AgileX Robotics
 * @copyright Copyright (c) 2024 Weston Robot Pte. Ltd.
 */

#ifndef TITAN_PARAMS_HPP
#define TITAN_PARAMS_HPP

namespace westonrobot {
struct TitanParams {
  static constexpr double track = 0.86
      /*0.56*/;  // in meter (left & right wheel distance) //
  static constexpr double wheelbase = 0.845
      /*0.89*/;  // in meter (front & rear wheel distance) //

  static constexpr double max_linear_speed = 3.0;            // in m/s
  static constexpr double max_angular_speed = 0.7853;        // in rad/s
  static constexpr double max_speed_cmd = 3.0;              // in rad/s
  static constexpr double max_steer_angle_ackermann = 3.67; // 40 degree
  static constexpr double min_turn_radius = 0.7837;
};
}  // namespace westonrobot
#endif  // TITAN_PARAMS_HPP

/**
* @file titan_messenger.cpp
* @date 2021-04-20
* @brief
*
# @copyright Copyright (c) 2021 AgileX Robotics
* @copyright Copyright (c) 2023 Weston Robot Pte. Ltd.
*/

#include "titan_base/titan_messenger.hpp"

#include <cmath>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include "titan_msgs/ActuatorState.h"
#include "titan_msgs/DriverState.h"
#include "titan_msgs/MotorState.h"
#include "titan_msgs/SystemState.h"

#include "titan_base/titan_params.hpp"
#include "titan_base/kinematics_model.hpp"

using namespace ros;
using namespace titan_msgs;

namespace westonrobot {
namespace {
double DegreeToRadian(double x) { return x * M_PI / 180.0; }
}  // namespace

///////////////////////////////////////////////////////////////////////////////////

TitanROSMessenger::TitanROSMessenger(ros::NodeHandle* nh) : nh_(nh) {
  LoadParameters();

  // connect to robot and setup ROS subscription
  if (robot_type_ == TitanSubType::kTitan) {
    robot_ = std::make_shared<TitanRobot>();
  } else {
    ROS_ERROR("Failed to init robot interface from sdk, robot type is wrong");
  }

  if (port_name_.find("can") != std::string::npos) {
    if (!robot_->Connect(port_name_)) {
      ROS_ERROR("Failed to connect to the CAN port");
      ros::shutdown();
    }
    robot_->EnableCommandedMode();
  } else {
    ROS_ERROR("Invalid port name: %s", port_name_.c_str());
    ros::shutdown();
  }

  SetupSubscription();
}

void TitanROSMessenger::Run() {
  ros::Rate rate(update_rate_);
  while (ros::ok()) {
    PublishStateToROS();
    ros::spinOnce();
    rate.sleep();
  }
}

void TitanROSMessenger::LoadParameters() {
  // load parameter from launch files
  nh_->param<std::string>("port_name", port_name_, std::string("can0"));
  nh_->param<std::string>("robot_model", robot_model_, std::string("titan"));
  nh_->param<std::string>("odom_frame", odom_frame_, std::string("odom"));
  nh_->param<std::string>("base_frame", base_frame_, std::string("base_link"));
  nh_->param<int>("update_rate", update_rate_, 50);
  nh_->param<std::string>("odom_topic_name", odom_topic_name_,
                          std::string("odom"));
  nh_->param<bool>("publish_odom_tf", publish_odom_tf_, false);

  ROS_INFO(
      "Successfully loaded the following parameters: \n port_name: %s\n "
      "robot_model: %s\n odom_frame: %s\n base_frame: %s\n "
      "update_rate: %d\n odom_topic_name: %s\n "
      "publish_odom_tf: %d\n",
      port_name_.c_str(), robot_model_.c_str(), odom_frame_.c_str(),
      base_frame_.c_str(), update_rate_, odom_topic_name_.c_str(),
      publish_odom_tf_);

  // load robot parameters
  if (robot_model_ == "titan") {
    robot_type_ = TitanSubType::kTitan;

    robot_params_.track = TitanParams::track;
    robot_params_.wheelbase = TitanParams::wheelbase;
    robot_params_.max_linear_speed = TitanParams::max_linear_speed;
    robot_params_.max_angular_speed = TitanParams::max_angular_speed;
    robot_params_.max_speed_cmd = TitanParams::max_speed_cmd;
    robot_params_.max_steer_angle_ackermann =
        TitanParams::max_steer_angle_ackermann;
    robot_params_.min_turn_radius = TitanParams::min_turn_radius;
  }
}

void TitanROSMessenger::SetupSubscription() {
  // publisher
  system_state_pub_ =
      nh_->advertise<titan_msgs::SystemState>("/system_state", 10);
  actuator_state_pub_ =
      nh_->advertise<titan_msgs::ActuatorStateArray>("/actuator_state", 10);
  odom_pub_ = nh_->advertise<nav_msgs::Odometry>(odom_topic_name_, 10);
  battery_state_pub_ =
      nh_->advertise<titan_msgs::BatteryState>("/battery_state", 10);

  // subscriber
  motion_cmd_sub_ = nh_->subscribe<geometry_msgs::Twist>(
      "/cmd_vel", 5, &TitanROSMessenger::TwistCmdCallback, this);
}

void TitanROSMessenger::PublishStateToROS() {
  current_time_ = ros::Time::now();

  static bool init_run = true;
  if (init_run) {
    last_time_ = current_time_;
    init_run = false;
    return;
  }

  auto state = robot_->GetRobotState();
  auto actuator_state = robot_->GetActuatorState();

  // update odometry
  {
    double dt = (current_time_ - last_time_).toSec();
    UpdateOdometry(state.motion_state.linear_velocity,
                   state.motion_state.steering_angle, dt);
    last_time_ = current_time_;
  }

  // publish system state
  {
    titan_msgs::SystemState system_msg;
    system_msg.header.stamp = current_time_;
    system_msg.vehicle_state = state.system_state.vehicle_state;
    system_msg.control_mode = state.system_state.control_mode;
    system_msg.error_code = state.system_state.error_code;
    system_msg.battery_voltage = state.system_state.battery_voltage;

    system_state_pub_.publish(system_msg);
  }

  // publish actuator state
  {
    titan_msgs::ActuatorStateArray actuator_msg;
    actuator_msg.header.stamp = current_time_;
    for (int i = 0; i < 4; i++) {
      titan_msgs::DriverState driver_state_msg;
      driver_state_msg.driver_voltage =
          actuator_state.actuator_ls_state->driver_voltage;
      driver_state_msg.driver_temperature =
          actuator_state.actuator_ls_state->driver_temp;
      driver_state_msg.motor_temperature =
          actuator_state.actuator_ls_state->motor_temp;
      driver_state_msg.driver_state =
          actuator_state.actuator_ls_state->driver_state;

      titan_msgs::MotorState motor_state_msg;
      motor_state_msg.rpm =
          actuator_state.actuator_hs_state->rpm;
      motor_state_msg.current =
          actuator_state.actuator_hs_state->current;
      motor_state_msg.pulse_count =
          actuator_state.actuator_hs_state->pulse_count;

      titan_msgs::ActuatorState actuator_state_msg;
      actuator_state_msg.id = i;
      actuator_state_msg.driver = driver_state_msg;
      actuator_state_msg.motor = motor_state_msg;

      actuator_msg.states.push_back(actuator_state_msg);
    }

    actuator_state_pub_.publish(actuator_msg);
  }

  // publish BMS state
  {
    auto common_sensor_state = robot_->GetCommonSensorState();

    titan_msgs::BatteryState batt_msg;
    batt_msg.header.stamp = current_time_;
    batt_msg.voltage = common_sensor_state.bms_basic_state.voltage;
    batt_msg.temperature = common_sensor_state.bms_basic_state.temperature;
    batt_msg.current = common_sensor_state.bms_basic_state.current;
    batt_msg.percentage = common_sensor_state.bms_basic_state.battery_soc;
    batt_msg.charge = std::numeric_limits<float>::quiet_NaN();
    batt_msg.capacity = std::numeric_limits<float>::quiet_NaN();
    batt_msg.design_capacity = std::numeric_limits<float>::quiet_NaN();
    batt_msg.power_supply_status =
        titan_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    batt_msg.power_supply_health =
        titan_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    batt_msg.power_supply_technology =
        titan_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    batt_msg.present = std::numeric_limits<uint8_t>::quiet_NaN();

    battery_state_pub_.publish(batt_msg);
  }
}

void TitanROSMessenger::UpdateOdometry(double linear, double steering_angle, double dt) {
  // update odometry calculations

  DualAckermanModel::state_type x = {position_x_, position_y_, theta_};
  DualAckermanModel::control_type u;
  u.v = linear;
  //for old driver
  // u.phi = -10 * steering_angle * (3.14/180);
  u.phi = steering_angle;
  // std::cout<<"control linear: "<<u.v<<" steer_angle: "<<u.phi<<std::endl;

  boost::numeric::odeint::integrate_const(
      boost::numeric::odeint::runge_kutta4<DualAckermanModel::state_type>(),
      DualAckermanModel(robot_params_.wheelbase, robot_params_.track, u), x,
      0.0, dt, (dt / 10.0));

  position_x_ = x[0];
  position_y_ = x[1];
  theta_ = x[2];
  

  // update odometry topics
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);

  // ROS_INFO("Pose: %f, %f, %f", position_x_, position_y_, theta_ / 3.14 *
  // 180.0);

  // publish odometry and tf messages
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = current_time_;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;

  odom_msg.pose.pose.position.x = position_x_;
  odom_msg.pose.pose.position.y = position_y_;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = odom_quat;


  odom_msg.twist.twist.linear.x = linear;
  odom_msg.twist.twist.linear.y = 0.0;
  if (steering_angle == 0) {
    odom_msg.twist.twist.angular.z = 0;
  } else {
    odom_msg.twist.twist.angular.z =
        (steering_angle / std::abs(steering_angle)) * 2 * linear /
        (robot_params_.wheelbase / std::abs(std::tan(steering_angle)) +
          robot_params_.track);
  }
  
  odom_pub_.publish(odom_msg);

  // // publish tf transformation
  if (publish_odom_tf_) {
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time_;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = position_x_;
    tf_msg.transform.translation.y = position_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_quat;

    tf_broadcaster_.sendTransform(tf_msg);
  }
}

void TitanROSMessenger::TwistCmdCallback(
    const geometry_msgs::Twist::ConstPtr& msg) {
  double steer_cmd;
  double radius;

  // judge if we lock the brake
  if(msg->linear.x != 0)
    robot_->ReleaseBrake();
  else
    robot_->ActivateBrake();
  
  // check for parking mode, only applicable to TitanMiniV2
    steer_cmd = CalculateSteeringAngle(*msg, radius);

  // send motion command to robot
  if (steer_cmd > robot_params_.max_steer_angle_ackermann) {
    steer_cmd = robot_params_.max_steer_angle_ackermann;
  }
  if (steer_cmd < -robot_params_.max_steer_angle_ackermann) {
    steer_cmd = -robot_params_.max_steer_angle_ackermann;
  }

  robot_->SetMotionCommand(msg->linear.x, steer_cmd);

}

double TitanROSMessenger::CalculateSteeringAngle(geometry_msgs::Twist msg,
                                                  double& radius) {
  double linear = std::abs(msg.linear.x);
  double angular = std::abs(msg.angular.z);

  // Circular motion
  radius = linear / angular;
  int k = (msg.angular.z * msg.linear.x) >= 0 ? 1.0 : -1.0;

  double l, w, phi_i;
  l = robot_params_.wheelbase;
  w = robot_params_.track;
  phi_i = atan((l / 2) / (radius - w / 2));
  // for old driver
  // phi_i = 0.1 * k * phi_i*(-180/3.14);
  phi_i = k * phi_i;
  ROS_INFO("command linear: %f, steer_angle: %f", linear, phi_i);
  return phi_i;
}
}  // namespace westonrobot
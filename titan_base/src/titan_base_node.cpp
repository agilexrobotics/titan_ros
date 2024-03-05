/**
* @file titan_base_node.cpp
* @date 2021-04-20
* @brief
*
# @copyright Copyright (c) 2021 AgileX Robotics
* @copyright Copyright (c) 2023 Weston Robot Pte. Ltd.
*/

#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

#include <memory>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include "titan_base/titan_messenger.hpp"
#include "ugv_sdk/details/robot_base/titan_base.hpp"

using namespace westonrobot;

std::shared_ptr<TitanRobot> robot;

void SignalHandler(int s)
{
  printf("Caught signal %d, program exit\n", s);
  exit(EXIT_FAILURE);
}

void controlSingal()
{
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = SignalHandler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
}

int main(int argc, char** argv)
{
  // setup ROS node
  ros::init(argc, argv, "titan_node");
  ros::NodeHandle node("~");

  controlSingal();

  // instantiate a robot object
  TitanROSMessenger messenger(&node);
  messenger.Run();

  return 0;
}

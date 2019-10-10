/*
 * navx_node.cpp
 * Runs the Kauai Labs NavX, using modified NavX library
 * VERSION: 1.0.0
 * Last changed: 2019-10-05
 * Authors: Jude Sauve <sauve031@umn.edu>
 * Maintainers: Nick Schatz <schat127@umn.edu>
 * MIT License
 * Copyright (c) 2019 UMN Robotics
 */

// ROS Libs
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>

// Native_Libs
#include <string>

// Custom_Libs
#include "ahrs/AHRS.h"

#define DEG_TO_RAD ((2.0 * 3.14159) / 360.0)

// ROS Node and Publishers
ros::NodeHandle * nh;
ros::NodeHandle * pnh;
ros::Publisher imu_pub;
ros::Publisher euler_pub;

// ROS Callbacks
void update_callback(const ros::TimerEvent&);

// ROS Params
double frequency;
bool publish_euler;
std::string device_path;
std::string frame_id;

// Global_Vars
AHRS* com;
int seq = 0;

int main(int argc, char** argv) {
  // Init ROS
  ros::init(argc, argv, "navx_node");
  nh = new ros::NodeHandle();
  pnh = new ros::NodeHandle("~");

  // Params
  pnh->param<double>("frequency", frequency, 50.0);
  pnh->param<bool>("publish_euler", publish_euler, false);
  pnh->param<std::string>("device_path", device_path, "/dev/ttyACM0");
  pnh->param<std::string>("frame_id", frame_id, "imu_link");

  // Init IMU
  com = new AHRS(device_path);

  // Subscribers
  ros::Timer update_timer = nh->createTimer(ros::Duration(1.0/frequency), update_callback);

  // Publishers
  imu_pub = nh->advertise<sensor_msgs::Imu>("imu/data", 10);
  euler_pub = nh->advertise<geometry_msgs::Point>("imu/euler", 10);

  // Spin
  ros::spin();
}

void update_callback(const ros::TimerEvent&) {
  // Publish IMU message

  sensor_msgs::Imu msg;
  msg.header.stamp = ros::Time::now();
  msg.header.seq = seq++;
  msg.header.frame_id = frame_id;
  
  msg.orientation.x = com->GetQuaternionX();
  msg.orientation.y = com->GetQuaternionY();
  msg.orientation.z = com->GetQuaternionZ();
  msg.orientation.w = com->GetQuaternionW();
  msg.orientation_covariance[0] = 0;
  msg.orientation_covariance[4] = 0;
  msg.orientation_covariance[8] = 0;
  
  msg.angular_velocity.x = com->GetPitchRate() * DEG_TO_RAD;
  msg.angular_velocity.y = com->GetRollRate() * DEG_TO_RAD;
  msg.angular_velocity.z = com->GetYawRate() * DEG_TO_RAD;
  msg.angular_velocity_covariance[0] = 0;
  msg.angular_velocity_covariance[4] = 0;
  msg.angular_velocity_covariance[8] = 0;
  
  msg.linear_acceleration.x = com->GetWorldLinearAccelX() * 9.81;
  msg.linear_acceleration.y = com->GetWorldLinearAccelY() * 9.81;
  msg.linear_acceleration.z = com->GetWorldLinearAccelZ() * 9.81;
  msg.linear_acceleration_covariance[0] = 0.8825985;
  msg.linear_acceleration_covariance[4] = 0.8825985;
  msg.linear_acceleration_covariance[8] = 1.569064;
  
  imu_pub.publish(msg);

  if (publish_euler) {
    // Publish Euler message
    geometry_msgs::Point euler;
    euler.x = com->GetRoll();
    euler.y = com->GetPitch();
    euler.z = com->GetYaw();
    euler_pub.publish(euler);
  }
}

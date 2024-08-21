/*
 * Copyright 2019 Daniel Koch, BYU MAGICC Lab
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file tracker_handler.cpp
 * @author Daniel Koch <daniel.p.koch@gmail.com>
 */

#include <optitrack_vrpn/tracker_handler.h>

#include <geometry_msgs/TransformStamped.h>

#include <cmath>
#include <sstream>

typedef int64_t s64;

/*!
 * Get the LCM URL with desired TTL.
 */
std::string getLcmUrl(s64 ttl) {
    assert(ttl >= 0 && ttl <= 255);
    return "udpm://239.255.76.67:7667?ttl=" + std::to_string(ttl);
}

namespace optitrack_vrpn
{

TrackerHandler::TrackerHandler(const std::string& name,
                               const TrackerHandlerOptions& options,
                               const std::shared_ptr<vrpn_Connection>& connection,
                               TimeManager& time_manager) :
  name_(name),
  options_(options),
  tf_child_frame_(name),
  tf_child_frame_ned_(name + "_ned"),
  connection_(connection),
  time_manager_(time_manager),
  tracker_((name + "@" + options.host).c_str(), connection_.get()),
  robot_states_lcm_publisher_(getLcmUrl(255))
{
    enu_msg_last_.pose.orientation.w = 1.0;
    enu_msg_last_.pose.orientation.x = 0.0;
    enu_msg_last_.pose.orientation.x = 0.0;
    enu_msg_last_.pose.orientation.x = 0.0;
    enu_msg_last_.pose.position.x = 0.0;
    enu_msg_last_.pose.position.y = 0.0;
    enu_msg_last_.pose.position.z = 0.0;
    stamp_last_ = ros::Time::now();

    rfu_to_flu_.setRPY(0.0, 0.0, M_PI_2); // rotate pi/2 (90deg) about z-axis

  std::string topic_name = sanitize_name(name_);

  enu_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_name + "_enu", 1);
    enu_ode_pub_ = nh_.advertise<nav_msgs::Odometry>(topic_name + "_odo_enu", 1);
    ned_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_name + "_ned", 1);

  tracker_.register_change_handler(this, &TrackerHandler::position_callback_wrapper);

    for (int i = 0; i < 3; ++i) {
        robot_state_estimator_lcm_.p[i] = 0.0;
        robot_state_estimator_lcm_.vWorld[i] = 0.0;
        robot_state_estimator_lcm_.vBody[i] = 0.0;
        robot_state_estimator_lcm_.rpy[i] = 0.0;
        robot_state_estimator_lcm_.omegaBody[i] = 0.0;
        robot_state_estimator_lcm_.omegaWorld[i] = 0.0;
        robot_state_estimator_lcm_.foot_p1[i] = 0.0;
        robot_state_estimator_lcm_.foot_p2[i] = 0.0;
        robot_state_estimator_lcm_.foot_p3[i] = 0.0;
        robot_state_estimator_lcm_.foot_p4[i] = 0.0;
    }
    robot_state_estimator_lcm_.p[2] = 0.38;
    for (int i = 0; i < 4; ++i) {
        robot_state_estimator_lcm_.quat[i] = 0.0;
    }
    robot_state_estimator_lcm_.quat[3] = 1.0;
}

void TrackerHandler::position_callback_wrapper(void *userData, vrpn_TRACKERCB info)
{
  TrackerHandler *handler = reinterpret_cast<TrackerHandler*>(userData);
  handler->position_callback(info);
}

void TrackerHandler::position_callback(const vrpn_TRACKERCB& info)
{
  ros::Time stamp = time_manager_.resolve_timestamp(info.msg_time);

  // axis mappings for ENU:
  //   East  (X): OptiTrack Z
  //   North (Y): OptiTrack X
  //   Up    (Z): OptiTrack Y
  geometry_msgs::PoseStamped enu_msg;
  enu_msg.header.stamp = stamp;
  enu_msg.header.frame_id = options_.frame;

  enu_msg.pose.position.x =  info.pos[VRPNIndex::Z];
  enu_msg.pose.position.y =  info.pos[VRPNIndex::X];
  enu_msg.pose.position.z =  info.pos[VRPNIndex::Y];

  // first get rotation to right-front-up body frame, then rotate to forward-left-up
  tf2::Quaternion enu_to_rfu(info.quat[VRPNIndex::Z], // x
                             info.quat[VRPNIndex::X], // y
                             info.quat[VRPNIndex::Y], // z
                             info.quat[VRPNIndex::W]); // w
  tf2::Quaternion quat = enu_to_rfu * rfu_to_flu_;

  enu_msg.pose.orientation.x =  quat.x();
  enu_msg.pose.orientation.y =  quat.y();
  enu_msg.pose.orientation.z =  quat.z();
  enu_msg.pose.orientation.w =  quat.w();
  enu_pub_.publish(enu_msg);
  send_transform(enu_msg, tf_child_frame_);

  // axis mappings for NED:
  //   North (X): OptiTrack X
  //   East  (Y): OptiTrack Z
  //   Down  (Z): OptiTrack -Y
  geometry_msgs::PoseStamped ned_msg;
  ned_msg.header.stamp = stamp;
  ned_msg.header.frame_id = options_.ned_frame;
  ned_msg.pose.position.x =  info.pos[VRPNIndex::X];
  ned_msg.pose.position.y =  info.pos[VRPNIndex::Z];
  ned_msg.pose.position.z = -info.pos[VRPNIndex::Y];
  ned_msg.pose.orientation.x =  info.quat[VRPNIndex::X];
  ned_msg.pose.orientation.y =  info.quat[VRPNIndex::Z];
  ned_msg.pose.orientation.z = -info.quat[VRPNIndex::Y];
  ned_msg.pose.orientation.w =  info.quat[VRPNIndex::W];
  ned_pub_.publish(ned_msg);
  send_transform(ned_msg, tf_child_frame_ned_);

    nav_msgs::Odometry enu_ode_msg;
    enu_ode_msg.header.stamp = stamp;
    enu_ode_msg.header.frame_id = options_.frame;

    enu_ode_msg.pose.pose.position.x =  info.pos[VRPNIndex::Z];
    enu_ode_msg.pose.pose.position.y =  info.pos[VRPNIndex::X];
    enu_ode_msg.pose.pose.position.z =  info.pos[VRPNIndex::Y];

    enu_ode_msg.pose.pose.orientation.x =  quat.x();
    enu_ode_msg.pose.pose.orientation.y =  quat.y();
    enu_ode_msg.pose.pose.orientation.z =  quat.z();
    enu_ode_msg.pose.pose.orientation.w =  quat.w();

    double stamp_delta_ = stamp.toSec() - stamp_last_.toSec();
    enu_ode_msg.twist.twist.linear.x = (enu_ode_msg.pose.pose.position.x - enu_msg_last_.pose.position.x) / stamp_delta_;
    enu_ode_msg.twist.twist.linear.y = (enu_ode_msg.pose.pose.position.y - enu_msg_last_.pose.position.y) / stamp_delta_;
    enu_ode_msg.twist.twist.linear.z = (enu_ode_msg.pose.pose.position.z - enu_msg_last_.pose.position.z) / stamp_delta_;
    enu_ode_pub_.publish(enu_ode_msg);

    robot_state_estimator_lcm_.p[0] = enu_ode_msg.pose.pose.position.x;
    robot_state_estimator_lcm_.p[1] = enu_ode_msg.pose.pose.position.y;
    robot_state_estimator_lcm_.p[2] = enu_ode_msg.pose.pose.position.z;
    robot_state_estimator_lcm_.quat[0] = enu_ode_msg.pose.pose.orientation.x;
    robot_state_estimator_lcm_.quat[1] = enu_ode_msg.pose.pose.orientation.y;
    robot_state_estimator_lcm_.quat[2] = enu_ode_msg.pose.pose.orientation.z;
    robot_state_estimator_lcm_.quat[3] = enu_ode_msg.pose.pose.orientation.w;

    robot_state_estimator_lcm_.vWorld[0] = enu_ode_msg.twist.twist.linear.x;
    robot_state_estimator_lcm_.vWorld[1] = enu_ode_msg.twist.twist.linear.y;
    robot_state_estimator_lcm_.vWorld[2] = enu_ode_msg.twist.twist.linear.z;
    robot_state_estimator_lcm_.omegaWorld[0] = 0.0;
    robot_state_estimator_lcm_.omegaWorld[1] = 0.0;
    robot_state_estimator_lcm_.omegaWorld[2] = 0.0;

    robot_states_lcm_publisher_.publish("robot_states_optitrack", &robot_state_estimator_lcm_);
    stamp_last_ = stamp;
    enu_msg_last_ = enu_msg;
}

void TrackerHandler::send_transform(const geometry_msgs::PoseStamped &pose, const std::string &child_frame)
{
  geometry_msgs::TransformStamped tf;

  tf.header.stamp = pose.header.stamp;
  tf.header.frame_id = pose.header.frame_id;
  tf.child_frame_id = child_frame;

  tf.transform.translation.x = pose.pose.position.x;
  tf.transform.translation.y = pose.pose.position.y;
  tf.transform.translation.z = pose.pose.position.z;

  tf.transform.rotation.x = pose.pose.orientation.x;
  tf.transform.rotation.y = pose.pose.orientation.y;
  tf.transform.rotation.z = pose.pose.orientation.z;
  tf.transform.rotation.w = pose.pose.orientation.w;

  tf_broadcaster_.sendTransform(tf);
}

std::string TrackerHandler::sanitize_name(const std::string& name)
{
  std::stringstream stream;

  bool first_character = true;
  for (char c : name)
  {
    if (isalnum(c) || c == '/')
    {
      stream << c;
    }
    else if (c == '_' && !first_character)
    {
      stream << c;
    }
    else if (c == ' ')
    {
      stream << '_';
    }

    first_character = false;
  }

  return stream.str();
}

} // namespace optitrack_vrpn

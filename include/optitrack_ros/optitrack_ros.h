/**
 * @file optitrack_ros.h
 * @author Daniel Koch <daniel.p.koch@gmail.com>
 */

#ifndef OPTITRACK_ROS_OPTITRACK_ROS_H
#define OPTITRACK_ROS_OPTITRACK_ROS_H

#include <ros/ros.h>
#include <optitrack_ros/tracker_handler.h>

#include "vrpn_Connection.h"

#include <memory>
#include <map>
#include <set>

namespace optitrack_ros
{

/**
 * @brief Manages the connection to Motive and initializes tracker handlers
 */
class OptiTrackROS
{
public:
  OptiTrackROS();
  ~OptiTrackROS();

private:
  std::set<std::string> sender_name_blacklist_ = std::set<std::string>({"VRPN Control"});

  std::string host_ = "192.168.1.186";

  std::shared_ptr<vrpn_Connection> connection_;
  std::map<std::string,TrackerHandler> trackers_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Timer mainloop_timer_;
  ros::Timer tracker_update_timer_;

  void mainloop_callback(const ros::TimerEvent& e);
  void tracker_update_callback(const ros::TimerEvent& e);
};

} // namespace optitrack_ros

#endif // OPTITRACK_ROS_OPTITRACK_ROS_H

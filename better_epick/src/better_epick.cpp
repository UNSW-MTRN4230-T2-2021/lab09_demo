/*
 * Copyright 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
   Desc: GazeboVacuumGripper plugin for manipulating objects in Gazebo
   Author: Kentaro Wada
   Date: 7 Dec 2015
   Modified by: Terry Lim
   Date: 15 August 2021
 */

#include <algorithm>
#include <assert.h>

#include <std_msgs/Bool.h>
#include "better_epick.h"


namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(BetterEPick);

////////////////////////////////////////////////////////////////////////////////
// Constructor
BetterEPick::BetterEPick()
{
  connect_count_ = 0;
  status_ = false;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
BetterEPick::~BetterEPick()
{
  //event::Events::DisconnectWorldUpdateBegin(update_connection_);
  update_connection_.reset();

  // Custom Callback Queue
  queue_.clear();
  queue_.disable();
  rosnode_->shutdown();
  callback_queue_thread_.join();

  delete rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void BetterEPick::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  ROS_INFO("Loading gazebo_ros_vacuum_gripper");

  // Set attached model;
  parent_ = _model;

  // Get the world name.
  world_ = _model->GetWorld();

  // load parameters
  robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("bodyName"))
  {
    ROS_FATAL("vacuum_gripper plugin missing <bodyName>, cannot proceed");
    return;
  }
  else
    link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();

  link_ = _model->GetLink(link_name_);
  if (!link_)
  {
    std::string found;
    physics::Link_V links = _model->GetLinks();
    for (size_t i = 0; i < links.size(); i++) {
      found += std::string(" ") + links[i]->GetName();
    }
    ROS_FATAL("gazebo_ros_vacuum_gripper plugin error: link named: %s does not exist", link_name_.c_str());
    ROS_FATAL("gazebo_ros_vacuum_gripper plugin error: You should check it exists and is not connected with fixed joint");
    ROS_FATAL("gazebo_ros_vacuum_gripper plugin error: Found links are: %s", found.c_str());
    return;
  }

  if (!_sdf->HasElement("topicName"))
  {
    ROS_FATAL("vacuum_gripper plugin missing <serviceName>, cannot proceed");
    return;
  }
  else
    topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Parameters for Vacuum Mechanism
  // max force: [N]
  if (_sdf->HasElement("maxForce")) {
    max_force_ = _sdf->GetElement("maxForce")->Get<double>();
  } else {
    max_force_ = 20;
  }
  // max distance to apply force: [m]
  if (_sdf->HasElement("maxDistance")) {
    max_distance_ = _sdf->GetElement("maxDistance")->Get<double>();
  } else {
    max_distance_ = 0.05;
  }
  // distance to apply friction: [m]
  if (_sdf->HasElement("minDistance")) {
    min_distance_ = _sdf->GetElement("minDistance")->Get<double>();
  } else {
    min_distance_ = 0.01;
  }

  rosnode_ = new ros::NodeHandle(robot_namespace_);

  // Custom Callback Queue
  ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<std_msgs::Bool>(
    topic_name_, 1,
    boost::bind(&BetterEPick::Connect, this),
    boost::bind(&BetterEPick::Disconnect, this),
    ros::VoidPtr(), &queue_);
  pub_ = rosnode_->advertise(ao);

  // Custom Callback Queue
  ros::AdvertiseServiceOptions aso1 =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
    "on", boost::bind(&BetterEPick::OnServiceCallback,
    this, _1, _2), ros::VoidPtr(), &queue_);
  srv1_ = rosnode_->advertiseService(aso1);
  ros::AdvertiseServiceOptions aso2 =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
    "off", boost::bind(&BetterEPick::OffServiceCallback,
    this, _1, _2), ros::VoidPtr(), &queue_);
  srv2_ = rosnode_->advertiseService(aso2);

  // Custom Callback Queue
  callback_queue_thread_ = boost::thread( boost::bind( &BetterEPick::QueueThread,this ) );

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&BetterEPick::UpdateChild, this));

  ROS_INFO("Loaded gazebo_ros_vacuum_gripper");
}

bool BetterEPick::OnServiceCallback(std_srvs::Empty::Request &req,
                                     std_srvs::Empty::Response &res)
{
  if (status_) {
    ROS_WARN("gazebo_ros_vacuum_gripper: already status is 'on'");
  } else {
    status_ = true;
    ROS_INFO("gazebo_ros_vacuum_gripper: status: off -> on");
  }
  return true;
}
bool BetterEPick::OffServiceCallback(std_srvs::Empty::Request &req,
                                     std_srvs::Empty::Response &res)
{
  if (status_) {
    status_ = false;
    ROS_INFO("gazebo_ros_vacuum_gripper: status: on -> off");
  } else {
    ROS_WARN("gazebo_ros_vacuum_gripper: already status is 'off'");
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void BetterEPick::UpdateChild()
{
  std_msgs::Bool grasping_msg;
  grasping_msg.data = false;
  if (!status_) {
    pub_.publish(grasping_msg);
    count = 0;
    return;
  }
  // apply force
  lock_.lock();
  ignition::math::Pose3d parent_pose = link_->WorldPose();
  physics::Model_V models = world_->Models();
  for (size_t i = 0; i < models.size(); i++) {
    if (models[i]->GetName() == link_->GetName() ||
        models[i]->GetName() == parent_->GetName())
    {
      continue;
    }
    physics::Link_V links = models[i]->GetLinks();
    for (size_t j = 0; j < links.size(); j++) {
      ignition::math::Pose3d link_pose = links[j]->WorldPose();
      ignition::math::Pose3d diff = parent_pose - link_pose;
      double norm = diff.Pos().Length();
      if (norm <= max_distance_) {
        // Record initial reference
        if (count++ < 1) {
          initial_parent_rot = parent_pose.Rot();
          initial_link_rot = link_pose.Rot();
        }
        links[j]->SetLinearAccel(link_->WorldLinearAccel());
        links[j]->SetAngularAccel(ignition::math::Vector3d(0, 0, 0));
        links[j]->SetLinearVel(link_->WorldLinearVel());
        links[j]->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
        // Update rotation of picked object as the gripper rotates.
        link_pose.Set(parent_pose.Pos(), parent_pose.Rot()*initial_parent_rot.Inverse()*initial_link_rot);
        links[j]->SetWorldPose(link_pose);
        grasping_msg.data = true;
        // Only pick up one item at a time.
        break;
      }
    }
  }
  pub_.publish(grasping_msg);
  lock_.unlock();
}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void BetterEPick::QueueThread()
{
  static const double timeout = 0.01;

  while (rosnode_->ok())
  {
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}

////////////////////////////////////////////////////////////////////////////////
// Someone subscribes to me
void BetterEPick::Connect()
{
  this->connect_count_++;
}

////////////////////////////////////////////////////////////////////////////////
// Someone subscribes to me
void BetterEPick::Disconnect()
{
  this->connect_count_--;
}

}  // namespace gazebo
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Luke Dennis */
#include "ros/ros.h"

#include "std_srvs/Empty.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/DeleteModel.h"

#include <sstream>
#include <fstream>


// This is for interfacing with Moveit move group
#include <moveit/move_group_interface/move_group_interface.h>

// These are for the various ROS message formats we need
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

constexpr double pi = 3.1415926535;

constexpr auto deg_to_rad(double deg) -> double {
    return deg * pi/180.0;
}

auto spawn_box(ros::NodeHandle n, geometry_msgs::Pose pose) -> bool;
auto delete_box(ros::NodeHandle n) -> bool;

static const std::string model_path = "/home/mtrn4230/lab_demo_repos/lab09_demo/lab09_gazebo/models/box/box.sdf";

/* The planning group can be found in the ur5e_epick_moveit_config/config/ur5e.srdf */
static const std::string PLANNING_GROUP = "manipulator";

int main(int argc, char** argv)
{
    //  ##### ROS setup #####

    ros::init(argc, argv, "lab09_demo");
    auto nh = ros::NodeHandle{};

    ros::ServiceClient onClient = nh.serviceClient<std_srvs::Empty>("/ur5e_epick/epick/on");
    ros::ServiceClient offClient = nh.serviceClient<std_srvs::Empty>("/ur5e_epick/epick/off");
    std_srvs::Empty srv;

    auto spinner = ros::AsyncSpinner(1);
    spinner.start();

    // ##### ensure gripper off and no box spawned #####
    offClient.call(srv);
    delete_box(nh);


    //  ##### MoveIt! setup #####
    auto move_group = moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);

    auto const* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    auto my_plan = moveit::planning_interface::MoveGroupInterface::Plan{};

    //  ##### State logging #####

    // We can print the name of the reference frame for this robot.
    ROS_INFO("Planning frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());

    // We can get a list of all the groups in the robot:
    {
        auto groups_str = std::string{};
        for(auto const & group : move_group.getJointModelGroupNames()) {
            groups_str += group + ", ";
        }
        ROS_INFO("Available Planning Groups: %s", groups_str.c_str());
    }


    // ##### Move to a "named" configuration (named in the srdf) #####
    ROS_INFO("Move to a \"named\" configuration (named in the srdf)");

    // Options are currently, "zero", "home" and "up"
    auto group_state = "home";
    move_group.setNamedTarget(group_state);
    // Check if plan is possible
    auto success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success) {
        ROS_WARN("Unable to plan path. Ensure goal pose is valid or adjust tolerance");
        return 1;
    }

    // Execute motion to home position
    ROS_INFO("Found path to %s, moving robot...", group_state);
    move_group.move();

    ROS_INFO("MOVE TO A POSE GOAL...");
    geometry_msgs::Pose target_pose1 = move_group.getCurrentPose().pose;
    target_pose1.position.z -= 0.1;
    move_group.setPoseTarget(target_pose1);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success) {
        ROS_WARN("Unable to plan path. Ensure goal pose is valid or adjust tolerance");
        return 1;
    }

    ROS_INFO("Found path down 10 cm, moving robot...");
    move_group.move();

    target_pose1 = move_group.getCurrentPose().pose;
    target_pose1.position.z += 0.1;
    move_group.setPoseTarget(target_pose1);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success) {
        ROS_WARN("Unable to plan path. Ensure goal pose is valid or adjust tolerance");
        return 1;
    }

    ROS_INFO("Found path up 10 cm, moving robot...");
    move_group.move();

    target_pose1 = move_group.getCurrentPose().pose;
    target_pose1.position.y += 0.1;
    move_group.setPoseTarget(target_pose1);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success) {
        ROS_WARN("Unable to plan path. Ensure goal pose is valid or adjust tolerance");
        return 1;
    }

    ROS_INFO("Found path 10cm in the negative y direction, moving robot...");
    move_group.move();

    target_pose1 = move_group.getCurrentPose().pose;
    target_pose1.position.y -= 0.1;
    move_group.setPoseTarget(target_pose1);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success) {
        ROS_WARN("Unable to plan path. Ensure goal pose is valid or adjust tolerance");
        return 1;
    }

    ROS_INFO("Found path 10 cm in the positive y direction, moving robot...");
    move_group.move();

    geometry_msgs::Pose start_pose2 =move_group.getCurrentPose().pose;
    std::vector<geometry_msgs::Pose> waypoints;

    geometry_msgs::Pose target_pose3 = start_pose2;

    target_pose3.position.z -= 0.1;
    waypoints.push_back(target_pose3);  // down

    target_pose3.position.y -= 0.1;
    waypoints.push_back(target_pose3);  // right

    target_pose3.position.z += 0.1;
    target_pose3.position.y += 0.1;
    target_pose3.position.x -= 0.1;
    waypoints.push_back(target_pose3);  // up and left

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    ROS_INFO("Executing Trajectory");
    move_group.execute(trajectory);


        // ##### Spawn box on table #####
    geometry_msgs::Pose box_pose;
    box_pose.position.x = 0.8 + 0.588; // Origin + offset calculated from URSim
    box_pose.position.y = 0 + 0.133;
    box_pose.position.z = 0.775 + 0.01;


    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY( 0, 0, 0 );

    box_pose.orientation = tf2::toMsg(myQuaternion);

    if(!spawn_box(nh,box_pose)) {
        return 1;
    }

    move_group.setNamedTarget(group_state);
    // Check if plan is possible
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success) {
        ROS_WARN("Unable to plan path. Ensure goal pose is valid or adjust tolerance");
        return 1;
    }

    // Execute motion to home position
    ROS_INFO("Found path to %s, moving robot...", group_state);
    move_group.move();

    target_pose1 = move_group.getCurrentPose().pose;
    target_pose1.position.z -= 0.15;
    move_group.setPoseTarget(target_pose1);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success) {
        ROS_WARN("Unable to plan path. Ensure goal pose is valid or adjust tolerance");
        return 1;
    }

    ROS_INFO("Found path down 15 cm, moving robot...");
    move_group.move();

    ROS_INFO("Turning on the gripper...");
    onClient.call(srv);




    target_pose1 = move_group.getCurrentPose().pose;
    target_pose1.position.z += 0.15;
    move_group.setPoseTarget(target_pose1);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success) {
        ROS_WARN("Unable to plan path. Ensure goal pose is valid or adjust tolerance");
        return 1;
    }

    ROS_INFO("Found path up 15 cm, moving robot...");
    move_group.move();

    target_pose1 = move_group.getCurrentPose().pose;
    target_pose1.position.x -= 0.7;
    target_pose1.position.y += 0.15;
    move_group.setPoseTarget(target_pose1);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success) {
        ROS_WARN("Unable to plan path. Ensure goal pose is valid or adjust tolerance");
        return 1;
    }

    ROS_INFO("Found path up 15 cm, moving robot...");
    move_group.move();


    ROS_INFO("Turning off the gripper...");
    offClient.call(srv);

    ros::shutdown();
    return 0;
}

auto const box_name = "spawnedBox";

auto delete_box(ros::NodeHandle n) -> bool {

    ros::ServiceClient deleteModel = n.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");
    deleteModel.waitForExistence();
    gazebo_msgs::DeleteModel srv;

    srv.request.model_name = box_name;

    deleteModel.call(srv);

    return srv.response.success;
}

// Author: Max Kelly
auto spawn_box(ros::NodeHandle n, geometry_msgs::Pose pose) -> bool {

    ros::ServiceClient spawnModel = n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    spawnModel.waitForExistence();
    gazebo_msgs::SpawnModel srv;

    srv.request.model_name = box_name;

    // load sdf file
    std::ifstream ifs;
    ifs.open(model_path);
    std::stringstream buffer;
    buffer << ifs.rdbuf();

    srv.request.model_xml = buffer.str();

    srv.request.initial_pose = pose;
    srv.request.robot_namespace = "/";
    srv.request.reference_frame = "world";

    spawnModel.call(srv);
    return srv.response.success;
}


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

// static const std::string model_path = "/home/raghav/lab_demo_repos/lab09_demo/lab09_gazebo/models/box/box.sdf";
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

    // ##### Move to a specific joint configuration #####
    ROS_INFO("Move to a specific joint configuration ");

    auto joint_group_positions = std::vector<double>{};
    // ensure the vector has a joint position for each joint
    move_group.getCurrentState()->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // Now, let's set it to a pre-calculated position just above the table
    joint_group_positions[0] = deg_to_rad(0);       // shoulder_pan_joint
    joint_group_positions[1] = deg_to_rad(-61.58);  // shoulder_lift_joint
    joint_group_positions[2] = deg_to_rad(104.42);  // elbow_joint
    joint_group_positions[3] = deg_to_rad(-132.84); // wrist_1_joint
    joint_group_positions[4] = deg_to_rad(-90);     // wrist_2_joint
    joint_group_positions[5] = deg_to_rad(0);       // wrist_3_joint

    // Set joint configuration as target
    move_group.setJointValueTarget(joint_group_positions);

    // Check if plan is possible
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success) {
        ROS_WARN("Unable to plan path. Rerun or adjust target pose or planning parameters");
        return 1;
    }

    // Execute motion
    ROS_INFO("Found path to given joint config, moving robot...");
    move_group.move();



    // ##### Save a record of the current pose for later reference #####
    ROS_INFO("Save a record of the current pose for later reference");

    move_group.rememberJointValues("above_table");



    // ##### Move through various named positions #####
    ROS_INFO("Move through various named positions");
    auto group_states = std::vector<std::string>{"zero", "home", "above_table", "up"};

    // Loop through states
    for(auto const & state : group_states) {
        move_group.setNamedTarget(state);

        // Check if plan is possible
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(!success) {
            ROS_WARN("Unable to plan path to %s. Ensure goal pose is valid or adjust tolerance", state.c_str());
            return 1;
        }

        // Execute motion to set position
        ROS_INFO("Found path to %s, moving robot...", state.c_str());
        move_group.move();
    }

    // ##### Move to home #####
    group_state = "home";
    move_group.setNamedTarget(group_state);

    // Check if plan is possible
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success) {
        ROS_WARN("Unable to plan path to %s. Ensure goal pose is valid or adjust tolerance", group_state);
        return 1;
    }

    // Execute motion to set position
    ROS_INFO("Found path to %s, moving robot...", group_state);
    move_group.move();


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

    // ##### Move to table #####
    group_state = "above_table";
    move_group.setNamedTarget(group_state);

    // Check if plan is possible
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success) {
        ROS_WARN("Unable to plan path to %s. Ensure goal pose is valid or adjust tolerance", group_state);
        return 1;
    }

    // Execute motion to set position
    ROS_INFO("Found path to %s, moving robot...", group_state);
    move_group.move();


    // ##### gripper on #####
    onClient.call(srv);


    // ##### Move up slightly #####

    move_group.getCurrentState()->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions[1] -= deg_to_rad(3);  // shoulder_lift_joint
    move_group.setJointValueTarget(joint_group_positions);

    move_group.setMaxVelocityScalingFactor(0.05);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success) {
        ROS_WARN("Unable to plan path. Ensure goal pose is valid or adjust tolerance");
        return 1;
    }

    // Execute motion to set position
    ROS_INFO("Found path, moving robot...");
    move_group.move();



    // ##### Move to an adjusted joint configuration #####
    ROS_INFO("Move to an adjusted joint configuration");

    // Set sholder_pan_joint to 90 deg
    move_group.setJointValueTarget("shoulder_pan_joint", deg_to_rad(90));

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success) {
        ROS_WARN("Unable to plan path. Rerun or adjust target pose or planning parameters");
        return 1;
    }

    // Execute motion
    move_group.move();


    // ##### Move down slightly #####

    move_group.getCurrentState()->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[1] += deg_to_rad(5);  // shoulder_lift_joint
    move_group.setJointValueTarget(joint_group_positions);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success) {
        ROS_WARN("Unable to plan path. Ensure goal pose is valid or adjust tolerance");
        return 1;
    }

    // Execute motion to set position
    ROS_INFO("Found path, moving robot...");
    move_group.move();


    // ##### gripper off #####
    offClient.call(srv);


    // ##### Move to home #####
    group_state = "home";
    move_group.setNamedTarget(group_state);
    move_group.setMaxVelocityScalingFactor(1);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success) {
        ROS_WARN("Unable to plan path. Rerun or adjust target pose or planning parameters");
        return 1;
    }

    // Execute motion
    move_group.move();

    


    // Delete box from Gazebo
    // Currently commented out so the box can be seen after the program finishes.
    // The box will be deleted when this node is started again
    // delete_box(nh);

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

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

/* The planning group can be found in the ur5e_epick_moveit_config/config/ur5e.srdf */
static const std::string PLANNING_GROUP = "manipulator";

int main(int argc, char** argv)
{
    //  ##### ROS setup #####

    ros::init(argc, argv, "move_group_interface_tutorial");
    auto nh = ros::NodeHandle{};
    auto spinner = ros::AsyncSpinner(1);
    spinner.start();

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

    // Print starting pose
    // move_group.getCurrentState()->printStateInfo();

    // ##### Move to a "named" configuration (named in the srdf) #####
    ROS_INFO("Move to a \"named\" configuration (named in the srdf)");

    // Options are currently, "zero", "home" and "up"
    auto group_state = "zero";
    move_group.setNamedTarget(group_state);

    // Check if plan is possible
    auto success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success) {
        ROS_WARN("Unable to plan path. Ensure goal pose is valid or adjust tolerance");
        return 1;
    }

    // Execute motion to zero position
    ROS_INFO("Found path to %s, moving robot...", group_state);
    move_group.move();


    // ##### Move to a specific joint configuration #####
    ROS_INFO("Move to a specific joint configuration ");

    auto joint_group_positions = std::vector<double>{};
    // ensure the vector has a joint position for each joint
    move_group.getCurrentState()->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // Now, let's set it to the approximate home position
    joint_group_positions[0] = deg_to_rad(   0);  // shoulder_pan_joint
    joint_group_positions[1] = deg_to_rad( -75);  // shoulder_lift_joint
    joint_group_positions[2] = deg_to_rad(  90);  // elbow_joint
    joint_group_positions[3] = deg_to_rad(-105);  // wrist_1_joint
    joint_group_positions[4] = deg_to_rad( -90);  // wrist_2_joint
    joint_group_positions[5] = deg_to_rad(   0);  // wrist_3_joint

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



    // ##### Save a record of the current pose for later reference #####
    ROS_INFO("Save a record of the current pose for later reference");

    move_group.rememberJointValues("new_home");


    // ##### Move through various named positions #####
    ROS_INFO("Move through various named positions");
    // auto group_states = std::vector<std::string>{"zero", "new_home", "up", "home"};
    auto group_states = std::vector<std::string>{"home"};

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

    auto pose = move_group.getCurrentPose().pose;
    std::cout << pose.position.x << " " << pose.position.y << " " << pose.position.z << "\n";
    for(auto const& rad : move_group.getCurrentRPY()) {
        std::cout << rad*180/pi << " ";
    }
    std::cout << "\n";

    // move_group.getCurrentState()->printStateInfo();

    // move_group.setRPYTarget(-pi,0,0);

    // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // if(!success) {
    //     ROS_WARN("Unable to plan path to RPY. Ensure goal pose is valid or adjust tolerance");
    //     return 1;
    // }

    // // Execute motion to set position
    // ROS_INFO("Found path to RPY, moving robot...");
    // move_group.move();





    // moveit_msgs::OrientationConstraint ocm;
    // // ocm.link_name = "epick_end_effector";
    // ocm.link_name = "tool0";
    // ocm.header.frame_id = "base_link";
    // ocm.orientation.x = -0.000351;
    // ocm.orientation.y = 0.703390;
    // ocm.orientation.z = 0.000134;
    // ocm.orientation.w = 0.710804;
    // // ocm.orientation = move_group.getCurrentPose().pose.orientation;
    // ocm.absolute_x_axis_tolerance = 0.1;
    // ocm.absolute_y_axis_tolerance = 0.1;
    // ocm.absolute_z_axis_tolerance = 0.1;
    // ocm.weight = 1.0;

    // std::cout << ocm.orientation;
    // // Now, set it as the path constraint for the group.
    // moveit_msgs::Constraints test_constraints;
    // test_constraints.orientation_constraints.push_back(ocm);
    // move_group.setPathConstraints(test_constraints);

    // // We will reuse the old goal that we had and plan to it.
    // // Note that this will only work if the current state already
    // // satisfies the path constraints. So, we need to set the start
    // // state to a new pose.
    // robot_state::RobotState start_state(*move_group.getCurrentState());
    // geometry_msgs::Pose start_pose2;
    // start_pose2.orientation.w = -1.0;
    // start_pose2.position.x = 0.55;
    // start_pose2.position.y = -0.35;
    // start_pose2.position.z = 0.5;
    // move_group.setStartState(*move_group.getCurrentState());

    // Now we will plan to the earlier pose target from the new
    // start state that we have just created.
    // move_group.setPoseTarget(move_group.getCurrentPose().pose);

    // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
    // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
    // move_group.setPlanningTime(10.0);

//     success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(!success) {
//         ROS_WARN("Unable to plan path. Rerun or adjust target pose or planning parameters");
//         return 1;
//     }

//     // Execute motion
//     move_group.move();

    //// Visualize the plan in RViz
    //visual_tools.deleteAllMarkers();
    //visual_tools.publishAxisLabeled(start_pose2, "start");
    //visual_tools.publishAxisLabeled(target_pose1, "goal");
    //visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
    //visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    //visual_tools.trigger();
    //visual_tools.prompt("next step");

    //// When done with the path constraint be sure to clear it.
    //move_group.clearPathConstraints();

    // Cartesian Paths
    // ^^^^^^^^^^^^^^^
    // You can plan a Cartesian path directly by specifying a list of waypoints
    // for the end-effector to go through. Note that we are starting
    // from the new start state above.  The initial pose (start state) does not
    // need to be added to the waypoint list but adding it can help with visualizations
    std::vector<geometry_msgs::Pose> waypoints;
    // waypoints.push_back(start_pose2);

    geometry_msgs::Pose target_pose = move_group.getCurrentPose().pose;
    waypoints.push_back(target_pose);  // start

    target_pose.position.z -= 0.05;
    waypoints.push_back(target_pose);  // down

    target_pose.position.y -= 0.2;
    waypoints.push_back(target_pose);  // right

    waypoints.push_back(move_group.getCurrentPose().pose);  // start

    // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
    // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
    // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
    move_group.setMaxVelocityScalingFactor(0.1);

    // We want the Cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in Cartesian
    // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
    // Warning - disabling the jump threshold while operating real hardware can cause
    // large unpredictable motions of redundant joints and could be a safety issue
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO("Cartesian path planned (%.2f%% acheived)", fraction * 100.0);

    // Execute motion through trajectory
    ROS_INFO("Found path through trajectory, moving robot...");
    move_group.execute(trajectory);

    //// Visualize the plan in RViz
    //visual_tools.deleteAllMarkers();
    //visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    //visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    //for (std::size_t i = 0; i < waypoints.size(); ++i)
    //  visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    //visual_tools.trigger();
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    ros::shutdown();
    return 0;
}

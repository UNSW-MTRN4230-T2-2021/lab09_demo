# Lab09 UR5e + epick with MoveIt! in Gazebo

## Setup

Ensure all necessary directories exist, clone this repository

Note: Use the HTTPS url instead if you still haven't set up your SSH keys

`https://github.com/UNSW-MTRN4230-T2-2021/lab09_demo.git`

```bash
mkdir -p ~/lab_workspaces/lab09_ws/src
mkdir -p ~/lab_demo_repos && cd ~/lab_demo_repos/
git clone git@github.com:UNSW-MTRN4230-T2-2021/lab09_demo
```


Link the packages into the workspace

```bash
cd ~/lab_workspaces/lab09_ws/src
ln -s ~/lab_demo_repos/lab09_demo/lab09_gazebo .
ln -s ~/lab_demo_repos/lab09_demo/ur5e_epick_moveit_config .
ln -s ~/ur5e_repos/fmauch_universal_robot/ur_description .
ln -s ~/ur5e_repos/fmauch_universal_robot/ur_kinematics .
```

Build the code

```bash
cd ~/lab_workspaces/lab09_ws
catkin_make
source devel/setup.bash
```

## Run the demo

Launch Gazebo and the MoveIt! planner so that Gazebo is ready to receive control
commands for the UR5e and the epick vacuum gripper.

```bash
roslaunch lab09_gazebo demo.launch
```

In a new terminal tab:

```bash
rosrun lab09_gazebo lab09_gazebo_moveit
```

## About the code

The program will first move the robot through several joint configurations.

It will then spawn a box, pick it up with the epick vacuum gripper, and drop it
off across the table.

It uses a new MoveIt! configuration generated for the UR5e + epick combination.

For a walk through of the code, you can watch a recording of Luke demoing the code and explaining it here: 
https://web.microsoftstream.com/video/046cebde-3e94-4e5f-aeae-936956d3b2ce

## Issues and limitations

### Issue: Warning about an unknown joint state:

> [WARN] [...] The complete state of the robot is not yet known. Missing epick_end_effector_joint

For the gripper to function, the `epick_end_effector_joint` must be `revolute`.
The warning relates to Gazebo/MoveIt not knowing the value of this joint as it
is actuated by the vacuum gripper plugin rather than the kinematics controller.

This warning has no effect on the operation of the program but may signal an
underlying issue that could be problematic in other use cases.

This may be a problem with my setup, if you find a way to fix this please open a PR
for me to review.

### Limitation: Joint based movement only

Currently there are no cartesian movements in the program.

### Limitation: Planning unaware of external world

Currently the robot is unaware of the table below it and will sometimes plan a
motion through the table and go into an unrecoverable state.

### Limitation: Epick force

Currently the epick is limited to objects under about 50g. If you need your object
to be heavier than this, you may need to look into increasing the maximum force
applied by the vacuum gripper.

### Box is spawned with fixed name

Only a single box can be spawned as the same name is used each time. Unique names
would need to be provided to spawn multiple boxes.

## Extensions to the code

Currently the code is fairly basic in that it only completes a set of predetermined
tasks. Look through the code and see how you can extend and refactor the code
so that it is more dynamic.

Some possible extensions include:

1. Create a service or set of services that takes in for example a group name,
joint configuration, or joint name and value and moves the robot to that location.
Change the node code so that it uses this service instead.
1. Create a service that can save the current configuration of the robot to a name
based on a given string and use this in the program instead of doing it manually.
1. Add cartesian motion to the program rather than moving to specific joint
configurations.
1. Add constraints on the robot so that it won't try to plan a path through the table.
1. Reset the world to a sane state if the robot collides with the table and
becomes unrecoverable.
1. Spawn boxes (or other shapes) with unique names and track these names for deletion.

## (Some of the) Helpful resources I used

Setting up the moveit config of a robot with an end effector:

- https://gramaziokohler.github.io/compas_fab/latest/examples/03_backends_ros/08_ros_create_moveit_package_from_custom_urdf.html

Setting up + controlling the Vacuum Gripper:

- https://github.com/ValerioMa/robotiq_tcp
- http://docs.ros.org/en/melodic/api/gazebo_plugins/html/classgazebo_1_1GazeboRosVacuumGripper.html

Controlling the robot from MoveIt!:

- http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/move_group_interface/move_group_interface_tutorial.html
- http://docs.ros.org/en/melodic/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html

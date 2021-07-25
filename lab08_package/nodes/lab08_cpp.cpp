#include "ros/ros.h"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


#include "std_msgs/String.h"
#include <std_msgs/Float64.h>

#include "trajectory_msgs/JointTrajectory.h"
#include "std_srvs/Empty.h" 
#include "gazebo_msgs/SpawnModel.h" 
#include "geometry_msgs/Pose.h" 
#include "geometry_msgs/Point.h" 
#include "geometry_msgs/Quaternion.h" 

#include <sstream>
#include <fstream>
#include <math.h>

#include <chrono>
#include <thread>

// Moves ur5e to sepcific join position (do not modify)
void move_ur5e_to(ros::Publisher pub, std::vector<double> pos, float duration_time, int wait_time) {


  clock_t begin_time = clock();
  while((float( clock () - begin_time ) /  CLOCKS_PER_SEC) < wait_time) {

    trajectory_msgs::JointTrajectory traj;

    traj.joint_names.push_back("elbow_joint");
    traj.joint_names.push_back("shoulder_lift_joint");
    traj.joint_names.push_back("shoulder_pan_joint");
    traj.joint_names.push_back("wrist_1_joint");
    traj.joint_names.push_back("wrist_2_joint");
    traj.joint_names.push_back("wrist_3_joint");

    trajectory_msgs::JointTrajectoryPoint points;
    points.positions = pos;
    points.time_from_start = ros::Duration(duration_time);


    // Set the points to the trajectory
    traj.header.stamp = ros::Time::now();
    traj.points.push_back(points);

    // Publish Message
    pub.publish(traj);
    ros::spinOnce();
  } 

}

void spawn_box(ros::NodeHandle n, geometry_msgs::Pose pose) {

    ros::ServiceClient spawnModel = n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    spawnModel.waitForExistence();
    gazebo_msgs::SpawnModel srv;

    srv.request.model_name = "spawnedCubeCpp";

    // load sdf file
    std::ifstream ifs;
    ifs.open("src/ur5e_gazebo/models/cube/cube.sdf");
    std::stringstream buffer;
    buffer << ifs.rdbuf();
    
    srv.request.model_xml = buffer.str();
  
    srv.request.initial_pose = pose;
    srv.request.robot_namespace = "/";
    srv.request.reference_frame = "world";

    spawnModel.call(srv);


}


int main(int argc, char **argv) {

    ros::init(argc, argv, "lab08_cpp");
    ros::NodeHandle n;

    geometry_msgs::Pose pose;
    pose.position.x = 1.71;
    pose.position.y = 0.1348;
    pose.position.z = 0.775;


    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY( 0, 0, 0 );

    pose.orientation = tf2::toMsg(myQuaternion);

    std::cout << "generating box (students to do)" << std::endl;
    spawn_box(n,pose);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    ros::Publisher joint_pub = n.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 10);

    std::vector<double> pos;
    
    std::cout << "moving to above box" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    pos.push_back(M_PI / 9.1);
    pos.push_back(-1 * (M_PI / 9.1));
    pos.push_back(0.0);
    pos.push_back(-1 * (M_PI / 2));
    pos.push_back(3 * (M_PI /2));
    pos.push_back(0.0);

    move_ur5e_to(joint_pub, pos, 1.0, 4);

    pos.clear();

    std::cout << "moving down to pick up box" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    pos.push_back(M_PI / 11.1);
    pos.push_back(-1 * (M_PI / 11.1));
    pos.push_back(0.0);
    pos.push_back(-1 * (M_PI / 2));
    pos.push_back(3 * (M_PI /2));
    pos.push_back(0.0);

    move_ur5e_to(joint_pub, pos, 1.0, 4);
    
    pos.clear();

    std::cout << "turning on gripper (students to do)" << std::endl;

    //INSERT CODE HERE TO TURN GRIPPER ON
    ros::ServiceClient onClient = n.serviceClient<std_srvs::Empty>("/ur5e/epick/on");

    std_srvs::Empty srv;
    onClient.call(srv);

    std::cout << "moving box" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    pos.push_back(M_PI / 4);
    pos.push_back(-1 * (M_PI / 4));
    pos.push_back(0.0);
    pos.push_back(-1 * (M_PI / 2));
    pos.push_back(3 * (M_PI /2));
    pos.push_back(0.0);

    move_ur5e_to(joint_pub, pos, 8.0, 15);

    pos.clear();

    std::cout << "moving block back down" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    pos.push_back(M_PI / 11.1);
    pos.push_back(-1 * (M_PI / 11.1));
    pos.push_back(0.0);
    pos.push_back(-1 * (M_PI / 2));
    pos.push_back(3 * (M_PI /2));
    pos.push_back(0.0);

    move_ur5e_to(joint_pub, pos, 8.0, 16);

    pos.clear();

    std::cout << "turning off gripper (students to do)" << std::endl;

    //INSERT CODE HERE TO TURN GRIPPER OFF
    ros::ServiceClient offClient = n.serviceClient<std_srvs::Empty>("/ur5e/epick/off");

    offClient.call(srv);

    std::cout << "moving robot away from block" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    pos.push_back(M_PI / 4);
    pos.push_back(-1 * (M_PI / 4));
    pos.push_back(0.0);
    pos.push_back(-1 * (M_PI / 2));
    pos.push_back(3 * (M_PI /2));
    pos.push_back(0.0);

    move_ur5e_to(joint_pub, pos, 1.0, 4);

    pos.clear();

  return 0;
}

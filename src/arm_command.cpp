#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit/move_group_interface/move_group.h>
#include "sensor_msgs/JointState.h"
#include <array>
#include <string>

moveit::planning_interface::MoveGroupInterface* move_fetch_ptr;

int getIndex(std::vector<std::string> v, std::string K)
{
    auto it = find(v.begin(), v.end(), K);
    if (it != v.end())
    {
        int index = it - v.begin();
	return index;
    }
    else {
	return -1;
    }
}

void jointCommandsCb(const sensor_msgs::JointState::ConstPtr& msg)
{
    std::vector<std::string> names = move_fetch_ptr->getJointNames();
    for(auto i: names){
    ROS_INFO_STREAM("NAME: " << i);
    }
    int index = getIndex(names, msg->name[0]);
    ROS_INFO_STREAM("SETTING INDEX: " << index);
    if(index == -1)
		return;

    std::vector<double> joints;
    joints = move_fetch_ptr->getCurrentJointValues();
    joints.at(index) = msg->position[0];
    move_fetch_ptr->setJointValueTarget(joints);

    //move_fetch_ptr->setJointValueTarget(msg->name[0], msg->position[0]);
    move_fetch_ptr->asyncMove();
}


int main(int argc, char** argv){
    ros::init(argc, argv, "fetch_arm_out");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/joint_commands", 1000, jointCommandsCb);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_fetch("arm_with_torso");
    move_fetch_ptr = &move_fetch;

    //ROS_INFO_STREAM("MOVED ARM OUT!");

    ros::spin();

    return 0;
}

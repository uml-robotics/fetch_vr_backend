#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Bool.h>
#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit/move_group_interface/move_group.h>
#include "geometry_msgs/PoseArray.h"
#include <array>
#include <tf/transform_listener.h>
#include <ros/callback_queue.h>
#include <tf/transform_broadcaster.h>

moveit::planning_interface::MoveGroupInterface* move_fetch_ptr;
tf::TransformListener* listener_;
moveit::planning_interface::MoveGroupInterface::Plan plan;

void jointCommandsCb(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    std::vector<geometry_msgs::Pose> poses;
    for (geometry_msgs::Pose p : msg->poses){
       tf::Transform waypoint_tf;
       tf::poseMsgToTF(p, waypoint_tf);

       tf::StampedTransform waypoint_stamped(waypoint_tf, ros::Time(0), msg->header.frame_id, "/goal");

       tf::StampedTransform gripper_to_wrist;
       listener_->lookupTransform("/gripper_link", "/wrist_roll_link", ros::Time(0), gripper_to_wrist);

       tf::Transform waypoint_to_wrist;
       waypoint_to_wrist = waypoint_stamped * gripper_to_wrist;

       geometry_msgs::Pose new_pose;
       tf::poseTFToMsg(waypoint_to_wrist, new_pose);

       poses.push_back(new_pose);
    }
    moveit_msgs::RobotTrajectory trajectory;
    move_fetch_ptr->setPoseReferenceFrame("map");
    move_fetch_ptr->computeCartesianPath(poses, 0.01, 0.0, trajectory, true);

    plan.trajectory_ = trajectory;
    move_fetch_ptr->execute(plan);
}

void confirmationCb(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data){
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "fetch_arm_goals_direct");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Subscriber goal_sub = nh.subscribe("/gripper_goal_execute", 3, jointCommandsCb);

    moveit::planning_interface::MoveGroupInterface move_fetch("arm_with_torso");
    move_fetch_ptr = &move_fetch;
    move_fetch_ptr->setGoalTolerance(0.01);
    tf::TransformListener listener;
    listener_ = &listener;

    ros::waitForShutdown();

    return 0;
}

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Bool.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "geometry_msgs/PoseArray.h"
#include <array>
#include <tf/transform_listener.h>
#include <ros/callback_queue.h>
#include <tf/transform_broadcaster.h>

moveit::planning_interface::MoveGroupInterface* move_fetch_ptr;
tf::TransformListener* listener_;
//ros::Publisher plan_pub;
moveit::planning_interface::MoveGroupInterface::Plan plan;

void jointCommandsCb(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    std::vector<geometry_msgs::Pose> poses;
    for (geometry_msgs::Pose p : msg->poses){
       tf::Transform waypoint_tf;
       tf::poseMsgToTF(p, waypoint_tf);

       tf::StampedTransform waypoint_stamped(waypoint_tf, ros::Time(0), msg->header.frame_id, "/goal");
       //br.sendTransform(waypoint_stamped);

       tf::StampedTransform gripper_to_wrist;
       listener_->lookupTransform("/gripper_link", "/wrist_roll_link", ros::Time(0), gripper_to_wrist);

       //br.sendTransform(tf::StampedTransform(gripper_to_wrist, ros::Time::now(), "/map", "gripper2Wrist"));


       tf::Transform waypoint_to_wrist;
       waypoint_to_wrist = waypoint_stamped * gripper_to_wrist;

       //br.sendTransform(tf::StampedTransform(waypoint_to_wrist, ros::Time::now(), "/map", "wrist_goal"));

       geometry_msgs::Pose new_pose;
       tf::poseTFToMsg(waypoint_to_wrist, new_pose);

       poses.push_back(new_pose);
    }
    moveit_msgs::RobotTrajectory trajectory;
    move_fetch_ptr->setPoseReferenceFrame("map");
    move_fetch_ptr->computeCartesianPath(poses, 0.01, 0.0, trajectory, true);
    //bool status = (move_fetch_ptr->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    plan.trajectory_ = trajectory;
    move_fetch_ptr->execute(plan);

    //plan_pub.publish(trajectory.joint_trajectory);

    /* static tf::TransformBroadcaster br;
    for (geometry_msgs::Pose p : msg->poses){
       tf::Transform waypoint_tf;
       tf::poseMsgToTF(p, waypoint_tf);

       tf::StampedTransform waypoint_stamped(waypoint_tf, ros::Time(0), msg->header.frame_id, "/goal");
       br.sendTransform(waypoint_stamped);

       tf::StampedTransform gripper_to_wrist;
       listener_->lookupTransform("/gripper_link", "/wrist_roll_link", ros::Time(0), gripper_to_wrist);

       br.sendTransform(tf::StampedTransform(gripper_to_wrist, ros::Time::now(), "/map", "gripper2Wrist"));


       tf::Transform waypoint_to_wrist;
       waypoint_to_wrist = waypoint_stamped * gripper_to_wrist;

       br.sendTransform(tf::StampedTransform(waypoint_to_wrist, ros::Time::now(), "/map", "wrist_goal"));

       geometry_msgs::PoseStamped new_pose;
       tf::poseTFToMsg(waypoint_to_wrist, new_pose.pose);

       move_fetch_ptr->setPoseTarget(new_pose.pose);
       ROS_INFO_STREAM("MAKING MY PLAN!!!!");
       bool status = (move_fetch_ptr->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
       plan_pub.publish(plan.trajectory_.joint_trajectory);
       //move_fetch_ptr->asyncExecute(plan);
       //move_fetch_ptr->asyncMove();
       ros::Duration(1.0).sleep();
    }*/
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
    //plan_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/joint_plan", 1000);

    moveit::planning_interface::MoveGroupInterface move_fetch("arm_with_torso");
    move_fetch_ptr = &move_fetch;
    move_fetch_ptr->setGoalTolerance(0.01);
    tf::TransformListener listener;
    listener_ = &listener;

    ros::waitForShutdown();

    return 0;
}

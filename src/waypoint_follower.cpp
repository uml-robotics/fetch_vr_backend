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
#include <list>
#include <std_msgs/UInt8.h>

moveit::planning_interface::MoveGroupInterface* move_fetch_ptr;
tf::TransformListener* listener_;
ros::Publisher plan_pub;
ros::Publisher waypoint_error_pub;
ros::Publisher is_done_pub;
moveit::planning_interface::MoveGroupInterface::Plan plan;
std::list<moveit::planning_interface::MoveGroupInterface::Plan> planList;

void jointCommandsCb(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    std::vector<geometry_msgs::Pose> poses;
    uint8_t currentIndex = 1;
    std_msgs::UInt8 indexMessage;
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
       moveit_msgs::RobotTrajectory trajectory;

        if(!planList.empty()){
            robot_state::RobotState start_state(*(move_fetch_ptr->getCurrentState()));

            robot_state::RobotState state(start_state);
            const std::vector<double> joints = planList.back().trajectory_.joint_trajectory.points.back().positions;
            state.setJointGroupPositions(move_fetch_ptr->getName(), joints);
            move_fetch_ptr->setStartState(state);
        }

        move_fetch_ptr->setPoseReferenceFrame("map");
        double percentage = move_fetch_ptr->computeCartesianPath(poses, 0.01, 0.0, trajectory, true);
        ROS_INFO_STREAM("PERCENTAGE COMPLETE: " << percentage);
        poses.clear(); // only want one waypoint in each pose vectorx
        if(percentage != 1.0){
            ROS_INFO_STREAM("WAYPOINT IS OUT OF RANGE: " << currentIndex);
	    indexMessage.data = currentIndex;
            waypoint_error_pub.publish(indexMessage);
            planList.clear();
            move_fetch_ptr->setStartStateToCurrentState();
            return;
        }
        plan.trajectory_ = trajectory;
        ROS_INFO_STREAM("ADDING TO LIST!");
        planList.push_back(plan);
        plan_pub.publish(plan.trajectory_.joint_trajectory);
        currentIndex++;
    }
}

void onNewTrajectory(const std_msgs::Bool::ConstPtr& msg){
    if(msg->data){
        ROS_INFO_STREAM("CLEARING LIST!");
        planList.clear();
        move_fetch_ptr->setStartStateToCurrentState();
    }
}

void confirmationCB(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data){
        if(planList.size() <= 0){
            ROS_INFO_STREAM("Plan is size 0!");
            return;
        }
        ROS_INFO_STREAM("Executing next trajectory!");
        move_fetch_ptr->execute(planList.front());
        planList.pop_front();
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "fetch_arm_goals");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Subscriber goal_sub = nh.subscribe("/gripper_goal", 1000, jointCommandsCb);
    ros::Subscriber confirmation_sub = nh.subscribe("/joint_plan_confirmation", 1000, confirmationCB);
    ros::Subscriber trajectory_signal_sub = nh.subscribe("/joint_plan_signal", 1000, onNewTrajectory);
    plan_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/joint_plan", 1000);
    is_done_pub = nh.advertise<std_msgs::Bool>("/is_joint_plan_sent", 1000);
    waypoint_error_pub = nh.advertise<std_msgs::UInt8>("/out_of_range_waypoints", 1000);

    moveit::planning_interface::MoveGroupInterface move_fetch("arm_with_torso");
    move_fetch_ptr = &move_fetch;
    move_fetch_ptr->setGoalTolerance(0.01);
    move_fetch_ptr->setMaxAccelerationScalingFactor(1.0);
    move_fetch_ptr->setMaxVelocityScalingFactor(1.0);
    tf::TransformListener listener;
    listener_ = &listener;

    ros::waitForShutdown();

    return 0;
}

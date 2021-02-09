#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Bool.h>
#include <list>

actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* client_ptr;
std::list<geometry_msgs::Pose> poses;

void navGoalCb(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    poses.clear();
    ROS_INFO_STREAM("NEW NAV GOAL!");
    for(geometry_msgs::Pose p : msg->poses)
    {
	poses.push_back(p);
    }
}

void confirmationCB(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data){
        if(poses.size() <= 0){
            ROS_INFO_STREAM("Plan is size 0!");
            return;
        }
        ROS_INFO_STREAM("Executing next trajectory!");
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.pose = poses.front();
	goal.target_pose.header.frame_id = "map";
        client_ptr->sendGoal(goal);

        poses.pop_front();
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "nav_node");
    ros::NodeHandle nh;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client("move_base");
    client_ptr = &client;
    ROS_INFO_STREAM("STARTING NAV NODE!");

    ros::Subscriber goal_sub = nh.subscribe("/navigation_goal", 1000, navGoalCb);
    ros::Subscriber confirmation_sub = nh.subscribe("/nav_confirmation", 1000, confirmationCB);

    ros::spin();

    return 0;
}

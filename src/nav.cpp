#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>

actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* client_ptr;

void navGoalCb(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    for(geometry_msgs::Pose p : msg->poses)
    {
        ROS_INFO_STREAM("NEW NAV GOAL!");
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.pose = p;
	goal.target_pose.header.frame_id = "map";
        client_ptr->sendGoal(goal);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "nav_node");
    ros::NodeHandle nh;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client("move_base");
    client_ptr = &client;
    ROS_INFO_STREAM("STARTING NAV NODE!");

    ros::Subscriber goal_sub = nh.subscribe("/navigation_goal", 1000, navGoalCb);

    ros::spin();

    return 0;
}

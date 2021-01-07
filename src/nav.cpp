#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>

actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* client_ptr;

void navGoalCb(const nav_msgs::Path::ConstPtr& msg)
{
    int i = 0;
    int n = sizeof(msg->poses)/sizeof(msg->poses[0]);
    ROS_INFO_STREAM("POSES: " + n);
    for(i = 0; i < n; i++)
    {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = msg->poses[i];
        client_ptr->sendGoal(goal);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "nav_node");
    ros::NodeHandle nh;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client("move_base");
    client_ptr = &client;

    ros::Subscriber goal_sub = nh.subscribe("/navigation_goal", 1000, navGoalCb);

    ros::spin();

    return 0;
}

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/GripperCommandGoal.h>

actionlib::SimpleActionClient<control_msgs::GripperCommandAction>* client_ptr;

void gripperGoalCb(const control_msgs::GripperCommandGoal::ConstPtr& msg)
{
    client_ptr->sendGoal(*msg);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "gripper_node");
    ros::NodeHandle nh;

    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> client("gripper_controller/gripper_action");
    client_ptr = &client;

    ros::Subscriber goal_sub = nh.subscribe("/gripper_command", 1000, gripperGoalCb);

    ros::spin();

    return 0;
}

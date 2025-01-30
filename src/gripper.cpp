#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/GripperCommandGoal.h>
#include <std_msgs/Float64.h>

actionlib::SimpleActionClient<control_msgs::GripperCommandAction>* client_ptr;

void gripperGoalCb(const control_msgs::GripperCommandGoal::ConstPtr& msg)
{
    client_ptr->sendGoal(*msg);
}

void gripperFloatGoalCb(std_msgs::Float64 msg)
{
	double dmsg = static_cast<double>(msg.data);
    control_msgs::GripperCommandGoal goalMsg;
    goalMsg.command.position = msg.data;//dmsg;
    client_ptr->sendGoal(goalMsg);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "gripper_node");
    ros::NodeHandle nh;

    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> client("gripper_controller/gripper_action");
    client_ptr = &client;

    ros::Subscriber goal_sub = nh.subscribe("/gripper_command", 1000, gripperGoalCb);
    ros::Subscriber std_goal_sub = nh.subscribe("/gripper_float", 1000, gripperFloatGoalCb);

    ros::spin();

    return 0;
}

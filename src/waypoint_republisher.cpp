#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

ros::Publisher pose_array_pub;

geometry_msgs::PoseArray _msg;

void poseArrayCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    // Republish the received PoseArray message
    //pose_array_pub.publish(*msg);
    _msg = *msg;
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_array_republisher");
    ros::NodeHandle nh;

    // Subscriber to the PoseArray topic
    ros::Subscriber pose_array_sub = nh.subscribe("/gripper_goal", 1000, poseArrayCallback);

    // Publisher for the PoseArray topic
    pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("/gripper_goal/current", 1000);

    while (ros::ok())
    {
        pose_array_pub.publish(_msg);
        ros::spinOnce();
    }

    return 0;
}
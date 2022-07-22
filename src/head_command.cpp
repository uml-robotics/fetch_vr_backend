#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

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

actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *client;

void headJointsCb(const sensor_msgs::JointState::ConstPtr& msg)
{
    if ( ! client->isServerConnected()) {
        ROS_INFO("[HeadCommand] waitForServer...");
        client->waitForServer();
        ROS_INFO("... done");
    }

    int tilt_index = getIndex(msg->name, "head_tilt_joint");
    int pan_index = getIndex(msg->name, "head_pan_joint");

    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory trajectory;
    trajectory.joint_names = {"head_tilt_joint", "head_pan_joint"};

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = {msg->position[tilt_index], msg->position[pan_index]};
    point.velocities = {msg->velocity[tilt_index], msg->velocity[pan_index]};
    point.accelerations = {0, 0};
    point.effort = {msg->effort[tilt_index], msg->effort[pan_index]};

    trajectory.points.push_back(point);
    goal.trajectory = trajectory;

    control_msgs::JointTolerance tolerance;
    tolerance.name = "head_tilt_joint";
    tolerance.position = 3.2;
    tolerance.velocity = 3.2;
    tolerance.acceleration = 1.0;
    goal.path_tolerance.push_back(tolerance);
    tolerance.name = "head_pan_joint";
    goal.path_tolerance.push_back(tolerance);

    tolerance.position = 0.1;
    goal.goal_tolerance.push_back(tolerance);
    tolerance.name = "head_tilt_joint";
    goal.goal_tolerance.push_back(tolerance);

    ros::Duration t(10.0);
    goal.goal_time_tolerance = t;

    client->sendGoal(goal);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "fetch_head_command");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/head_command", 1000, headJointsCb);
    client = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("head_controller/follow_joint_trajectory", true);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::waitForShutdown();

    return 0;
}

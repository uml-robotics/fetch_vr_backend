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

void torsoJointCB(const sensor_msgs::JointState::ConstPtr& msg)
{
    if ( ! client->isServerConnected()) {
        ROS_INFO("[TorsoCommand] waitForServer...");
        client->waitForServer();
        ROS_INFO("... done");
    }

    int torso_index = getIndex(msg->name, "torso_lift_joint");

    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory trajectory;
    trajectory.joint_names = {"torso_lift_joint"};

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = {msg->position[torso_index]};
//    point.velocities = {msg->velocity[torso_index]};
//    point.accelerations = {0};
//    point.effort = {msg->effort[torso_index]};

    trajectory.points.push_back(point);
    goal.trajectory = trajectory;

    control_msgs::JointTolerance tolerance;
    tolerance.name = "torso_lift_joint";
    tolerance.position = 0.5;
    tolerance.velocity = 1.0;
    tolerance.acceleration = 1.0;

    goal.path_tolerance.push_back(tolerance);

    tolerance.position = 0.1;
    goal.goal_tolerance.push_back(tolerance);

    ros::Duration t(10.0);
    goal.goal_time_tolerance = t;

    client->sendGoal(goal);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "fetch_torso_command");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/torso_command", 1000, torsoJointCB);
    client = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("torso_controller/follow_joint_trajectory", true);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::waitForShutdown();

    return 0;
}

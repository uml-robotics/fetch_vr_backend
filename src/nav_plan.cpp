#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Bool.h>
#include <list>

#include <nav_msgs/GetPlan.h>
#include <tf/transform_listener.h>
#include <string>
#include <std_msgs/Int32.h>

actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* client_ptr;
std::list<geometry_msgs::Pose> poses;

ros::ServiceClient navService;
tf::TransformListener* listener_;

ros::Publisher plan_pub;
ros::Publisher waypoint_error_pub;

void navGoalCb(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    poses.clear();

    int currentIndex = 0;
    std_msgs::Int32 indexMessage;

    ROS_INFO_STREAM("NEW NAV GOAL!");
    bool gotone = false;
    for(geometry_msgs::Pose p : msg->poses)
    {
        nav_msgs::GetPlan req;
        if(!gotone){
            ROS_INFO_STREAM("FIRST!");
            tf::StampedTransform tf;
            listener_->lookupTransform("/map","/base_link",ros::Time(0),tf);
            geometry_msgs::Pose start_pose;
            tf::poseTFToMsg(tf, start_pose);
            geometry_msgs::PoseStamped start_posestamped;
            start_posestamped.header.frame_id = "map";
            start_posestamped.pose = start_pose;
            ROS_INFO_STREAM("START POSE: " << start_pose.position);
            req.request.start = start_posestamped;
            gotone = true;
        }
        else{
            ROS_INFO_STREAM("NOT FIRST!");
            geometry_msgs::PoseStamped start_posestamped;
            start_posestamped.header.frame_id = "map";
            start_posestamped.pose = poses.back();
            ROS_INFO_STREAM("START POSE: " << poses.back().position);
            req.request.start = start_posestamped;
        }
        geometry_msgs::PoseStamped goal_posestamped;
        goal_posestamped.header.frame_id = "map";
        goal_posestamped.pose = p;
        req.request.goal = goal_posestamped;

        req.request.tolerance = 0.1;
        ROS_INFO_STREAM("REQ START POSE: " << req.request.start.pose.position);

        navService.call(req);
	if(req.response.plan.poses.empty()) {
            ROS_INFO_STREAM("WAYPOINT IS OUT OF RANGE: " << currentIndex);
            indexMessage.data = currentIndex;
            waypoint_error_pub.publish(indexMessage);
            poses.clear();
            return;
        }


        plan_pub.publish(req.response.plan);
	poses.push_back(p);
        currentIndex++;
    }
    indexMessage.data = -1;
    waypoint_error_pub.publish(indexMessage);
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

    std::string serviceName = "move_base/make_plan";

    while(!ros::service::waitForService(serviceName)){
        ROS_INFO_STREAM("[nav_plan][main]: WAITING FOR SERVICE: " << serviceName << "!");
        ros::spinOnce();
    }

    navService = nh.serviceClient<nav_msgs::GetPlan>(serviceName);
    //navService = nh.serviceClient<nav_msgs::GetPlan>(serviceName, true);
    if(!navService){
      ROS_ERROR("Could not init service: %s!", navService.getService().c_str());
      return -1;
    }

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client("move_base");
    client_ptr = &client;
    ROS_INFO_STREAM("STARTING NAV NODE!");

    ros::Subscriber goal_sub = nh.subscribe("/navigation_goal", 1000, navGoalCb);
    ros::Subscriber confirmation_sub = nh.subscribe("/nav_confirmation", 1000, confirmationCB);
    plan_pub = nh.advertise<nav_msgs::Path>("/nav_path", 1000);
    waypoint_error_pub = nh.advertise<std_msgs::Int32>("/nav_out_of_range_waypoints", 1000);

    tf::TransformListener listener;
    listener_ = &listener;

    ros::spin();

    return 0;
}

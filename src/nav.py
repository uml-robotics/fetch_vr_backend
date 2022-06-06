#! /usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray
from spot_msgs.msg import TrajectoryAction, TrajectoryGoal
import tf2_ros
import actionlib


class Nav:
    def __init__(self):
        self.poses = []
        self.result_pub = rospy.Publisher("/spot_nav_result", Bool, queue_size=10)
        rospy.Subscriber("/nav_confirmation", Bool, self.confirmation_cb)
        rospy.Subscriber("/navigation_goal", PoseArray, self.nav_goal_cb)

    def nav_goal_cb(self, msg):
        # unpack the pose array, add each pose to the array poses
        self.poses = msg.poses

    def confirmation_cb(self, msg):
        if msg.data:
            # pop front pose off of poses
            if len(self.poses) <= 0:
                rospy.logerr("[SPOT_NAV]: Confirmation sent but no goal poses currently queued")
                return
            # send pose to the spot-ros handle trajectory server
            # Creating action client
            client = actionlib.SimpleActionClient('/spot/trajectory', TrajectoryAction)

            # Waiting for action server
            client.wait_for_server()

            # Creating and setting parameters of the action goal
            trajectory_goal = TrajectoryGoal()
            pose = self.poses.pop(0)
            pose.frame_id = "vision"
            tf_buffer = tf2_ros.Buffer()
            trajectory_goal.target_pose = tf_buffer.transform(pose, "body", rospy.Duration(1))
            trajectory_goal.target_pose = pose
            trajectory_goal.precise_positioning = True
            trajectory_goal.duration.data = rospy.Duration(2)

            # send goal to the action server, wait for result, print result to console
            client.send_goal(trajectory_goal)
            client.wait_for_result()
            result = client.get_result()
            rospy.loginfo(result)

            # Send result to unity
            self.result_pub.publish(result.success)


if __name__ == "__main__":
    rospy.init_node('fetch_backend')
    spotNav = Nav()
    rospy.loginfo("[SPOT_NAV]: Backend up and running!")
    rospy.spin()

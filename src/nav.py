#! /usr/bin/env python2
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray
from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import PoseStamped as GeoPoseStamped
import tf2_ros
import actionlib


class Nav:
    def __init__(self):
        self.poses = []
        self.result_pub = rospy.Publisher("/nav_goal_pose", GeoPoseStamped, queue_size=10)
        rospy.Subscriber("/nav_confirmation", Bool, self.confirmation_cb)
        rospy.Subscriber("/navigation_goal", PoseArray, self.nav_goal_cb)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def nav_goal_cb(self, msg):
        # unpack the pose array, add each pose to the array poses
        rospy.loginfo("[SPOT_NAV]: Recieved nav goal pose array of length {}".format(len(msg.poses)))
        self.poses = msg.poses

    def confirmation_cb(self, msg):
        if msg.data:
            # pop front pose off of poses
            if len(self.poses) <= 0:
                rospy.logerr("[SPOT_NAV]: Confirmation sent but no goal poses currently queued")
                return
            pose = self.poses.pop(0)


            ps = PoseStamped()
            ps.pose = pose
            ps.header.frame_id = "odom"
            ps = self.tf_buffer.transform(ps, "body", rospy.Duration(1))
            gps = GeoPoseStamped()
            gps.header = ps.header
            gps.pose = ps.pose
            self.result_pub.publish(gps)


if __name__ == "__main__":
    rospy.init_node('fetch_backend')
    spotNav = Nav()
    rospy.loginfo("[SPOT_NAV]: Backend up and running!")
    rospy.spin()

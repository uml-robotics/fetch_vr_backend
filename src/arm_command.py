#! /usr/bin/env python2
import rospy
from geometry_msgs.msg import PoseArray
from tf2_geometry_msgs import PoseStamped
from std_msgs.msg import Bool
import tf2_ros
from spot_msgs.srv import HandPose, HandPoseRequest


class SpotArmCommand:
    def __init__(self):
        self.poses = []

        self.result_pub = rospy.Publisher("/spot_manip_result", Bool, queue_size=10)
        rospy.Subscriber("/gripper_goal", PoseArray, self.goal_cb)
        rospy.Subscriber("/joint_plan_confirmation", Bool, self.confirmation_cb)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.wait_for_service("/spot/gripper_pose")
        self.pose_gripper = rospy.ServiceProxy("/spot/gripper_pose", HandPose)

        rospy.loginfo("[SPOT_MANIP]: Finished configuring!")

    def goal_cb(self, msg):
        self.poses.clear()
        for pose in msg.poses:
            self.poses.append(pose)

    def confirmation_cb(self, msg):
        if msg.data:
            # pop front pose off of poses
            if len(self.poses) <= 0:
                rospy.logerr("[SPOT_MANIP]: Confirmation sent but no goal poses currently queued")
                return

            # Convert the pose into the body frame
            pose = self.poses.pop(0)
            ps = PoseStamped()
            ps.pose = pose
            ps.header.frame_id = "odom"
            ps = self.tf_buffer.transform(ps, "body", rospy.Duration(1))

            # Send the pose to the hand pose server
            req = HandPoseRequest()
            req.pose_point = [ps.pose.position.x, ps.pose.position.y, ps.pose.position.z,
                              ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z,
                              ps.pose.orientation.w]
            resp = self.pose_gripper(req)
            self.result_pub.publish(resp.success)


if __name__ == "__main__":
    rospy.init_node("spot_arm_command")
    SpotArmCommand()
    rospy.spin()

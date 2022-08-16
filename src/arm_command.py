#! /usr/bin/env python2
import rospy
from geometry_msgs.msg import PoseArray, Transform
from geometry_msgs.msg import PoseStamped as PS
from tf2_geometry_msgs import PoseStamped
from std_msgs.msg import Bool
import tf2_ros
import tf
from spot_msgs.srv import HandPose, HandPoseRequest


class SpotArmCommand:
    def __init__(self):
        self.poses = []

        self.result_pub = rospy.Publisher("/spot_manip_result", Bool, queue_size=10)
        rospy.Subscriber("/gripper_goal", PoseArray, self.goal_cb)
        rospy.Subscriber("/joint_plan_confirmation", Bool, self.confirmation_cb)
        self.test = rospy.Publisher("/pose_test", PoseArray, queue_size=10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.br = tf.TransformBroadcaster()
        rospy.wait_for_service("/spot/gripper_pose")
        self.pose_gripper = rospy.ServiceProxy("/spot/gripper_pose", HandPose)
        rospy.loginfo("[SPOT_MANIP]: Finished configuring!")
        self.hand_control_sub = rospy.Subscriber("/hand_goal", PS, self.hand_pose_cb)

    def hand_pose_cb(self, msg):
        msg.header.frame_id = "hand"
        ps = self.tf_buffer.transform(msg, "flat_body", timeout=rospy.Duration(10))
        req = HandPoseRequest()
        req.pose_point = [ps.pose.position.x, ps.pose.position.y, ps.pose.position.z,
                          ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z,
                          ps.pose.orientation.w]
        resp = self.pose_gripper(req)

    def goal_cb(self, msg):
        self.poses = []
        for pose in msg.poses:
            self.poses.append(pose)
        # while not rospy.is_shutdown():
        #      msg.header.stamp = rospy.Time.now()
        #      self.test.publish(msg)

    def confirmation_cb(self, msg):
        if msg.data:
            # pop front pose off of poses
            if len(self.poses) <= 0:
                rospy.logerr("[SPOT_MANIP]: Confirmation sent but no goal poses currently queued")
                return

            # Convert the pose into the body frame
            pose = self.poses.pop(0)
            # ps = PoseStamped()
            # ps.pose = pose
            # ps.header.frame_id = "odom"
            # rospy.loginfo(ps.pose.orientation)
            # ps = self.tf_buffer.transform(ps, "flat_body", timeout=rospy.Duration(10))
            # rospy.loginfo(ps.pose.orientation)
            # # Send the pose to the hand pose server
            req = HandPoseRequest()
            req.pose_point = [pose.position.x, pose.position.y, pose.position.z,
                              pose.orientation.x, pose.orientation.y, pose.orientation.z,
                              pose.orientation.w]
            # Set wrist_tform_tool to be the location of /gripper in the /link_wr1 frame
            # wrist_tform_tool = self.tf_buffer.lookup_transform('gripper', 'link_wr1', rospy.Time())
            # wrist_tform_tool_array = [wrist_tform_tool.transform.translation.x,
            #                           wrist_tform_tool.transform.translation.y,
            #                           wrist_tform_tool.transform.translation.z, wrist_tform_tool.transform.rotation.x,
            #                           wrist_tform_tool.transform.rotation.y, wrist_tform_tool.transform.rotation.z,
            #                           wrist_tform_tool.transform.rotation.w]
            # rospy.loginfo(wrist_tform_tool_array)
            # req.wrist_tform_tool = wrist_tform_tool_array
            resp = self.pose_gripper(req)
            if not resp.success:
                rospy.logerr("[SPOT_MANIP]: Failed moving robot arm to pose {}, {}, {}".format(req.pose_point[0], req.pose_point[1], req.pose_point[2]))
            else:
                rospy.loginfo("[SPOT_MANIP]: Success moving robot arm to pose {}, {}, {}".format(req.pose_point[0], req.pose_point[1], req.pose_point[2]))
            b = Bool()
            b.data = resp.success
            self.result_pub.publish(b)


if __name__ == "__main__":
    rospy.init_node("spot_arm_command")
    spot_arm = SpotArmCommand()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        while True:
            try:
                origin = spot_arm.tf_buffer.lookup_transform("gripper", "hand", rospy.Time())
                pos = (origin.transform.translation.x,
                       origin.transform.translation.y,
                       origin.transform.translation.z)
                rot = (origin.transform.rotation.x,
                       origin.transform.rotation.y,
                       origin.transform.rotation.z,
                       origin.transform.rotation.w)
                spot_arm.br.sendTransform(pos, rot, rospy.Time.now(), "ui_gripper_origin", "gripper")
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
            rate.sleep()

    rospy.spin()

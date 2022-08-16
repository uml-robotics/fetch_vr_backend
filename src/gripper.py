#! /usr/bin/env python2
import actionlib
import rospy
from control_msgs.msg import GripperCommandGoal, GripperCommandActionResult
import math
from spot_msgs.msg import GripperAngleAction, GripperAngleResult, GripperAngleGoal
from spot_msgs.srv import GripperAngleMove, GripperAngleMoveRequest


class Gripper:
    def __init__(self):
        rospy.Subscriber("/gripper_command", GripperCommandGoal, self.gripper_command_cb)
        rospy.wait_for_service("/spot/gripper_angle_open")
        self.gripper_client = actionlib.SimpleActionClient('/spot/gripper_angle', GripperAngleAction)
        self.gripper_client.wait_for_server()
        self.gripper_command = rospy.ServiceProxy("/spot/gripper_angle_open", GripperAngleMove)

        self.gripper_result_pub = rospy.Publisher("/gripper_controller/gripper_action/result",
                                                  GripperCommandActionResult, queue_size=10)
        self.commands = []
        rospy.loginfo("[SPOT_GRIPPER]: finished configuring!")

    def gripper_command_cb(self, msg):
        r = math.radians(-900 * msg.command.position)
        self.commands.append(r)


if __name__ == "__main__":
    rospy.init_node("gripper_command_node")
    gripper = Gripper()
    while not rospy.is_shutdown():
        if len(gripper.commands) > 0:
            while len(gripper.commands) > 0:
                radians = gripper.commands.pop(0)
                # Publish and log the result to console
                gripper.gripper_client.send_goal(GripperAngleGoal(gripper_angle=radians))
                gripper.gripper_client.wait_for_result()
                resp = gripper.gripper_client.get_result()
                msg = GripperCommandActionResult()
                msg.result.reached_goal = resp.success
                gripper.gripper_result_pub.publish(msg)
                if resp.success:
                    rospy.loginfo("[SPOT_GRIPPER]: Success moving gripper! Radians: {}".format(radians))
                else:
                    rospy.logerr("[SPOT_GRIPPER]: Failed moving gripper! Radians: {}".format(radians))
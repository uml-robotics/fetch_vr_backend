#! /usr/bin/env python2
import rospy
from control_msgs.msg import GripperCommandGoal, GripperCommand
import math
from spot_msgs.srv import GripperAngleMove, GripperAngleMoveRequest


class Gripper:
    def __init__(self):
        rospy.Subscriber("/gripper_command", GripperCommandGoal, self.gripper_command_cb)
        rospy.wait_for_service("/spot/gripper_angle_open")
        self.gripper_command = rospy.ServiceProxy("/spot/gripper_angle_open", GripperAngleMove)
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
                resp = gripper.gripper_command(GripperAngleMoveRequest(gripper_angle=radians))
                if resp.success:
                    rospy.loginfo("[SPOT_GRIPPER]: Success moving gripper! Radians: {}".format(radians))
                else:
                    rospy.logerr("[SPOT_GRIPPER]: " + resp.message + " Radians: {}".format(radians))
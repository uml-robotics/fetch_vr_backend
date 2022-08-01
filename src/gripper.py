#! /usr/bin/env python2
import rospy
from control_msgs.msg import GripperCommand
import math
from spot_msgs.srv import GripperAngleMove, GripperAngleMoveRequest


class Gripper:
    def __init__(self):
        rospy.Subscriber("/gripper_command", GripperCommand, self.gripper_command_cb)
        rospy.wait_for_service("/spot/gripper_angle_open")
        self.gripper_command = rospy.ServiceProxy("/spot/gripper_angle_open", GripperAngleMove)
        rospy.loginfo("[SPOT_GRIPPER]: finished configuring!")

    def gripper_command_cb(self, msg):
        radians = math.radians(-900 * msg.command.position)
        resp = self.gripper_command(GripperAngleMoveRequest(gripper_angle=radians))


def main():
    Gripper()
    rospy.spin()


if __name__ == "__main__":
    main()
#! /usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from std_msgs.msg import String
import actionlib
from spot_msgs.msg import TrajectoryAction, TrajectoryGoal, TrajectoryResult
from geometry_msgs.msg import PoseStamped


class SpotControl:
    def __init__(self):
        rospy.Subscriber("/spot_control", String, self.control_update_cb)
        self.result_pub = rospy.Publisher("/spot_control_result", TriggerResponse, queue_size=1)
        rospy.wait_for_service("/spot/claim")
        rospy.wait_for_service("/spot/power_on")
        rospy.wait_for_service("/spot/sit")
        rospy.wait_for_service("/spot/stand")
        rospy.wait_for_service("/spot/stop")
        rospy.wait_for_service("/spot/power_off")
        rospy.wait_for_service("/spot/release")
        self.claim = rospy.ServiceProxy("/spot/claim", Trigger)
        self.power_on = rospy.ServiceProxy("/spot/power_on", Trigger)
        self.dock = rospy.ServiceProxy("/spot/dock", Trigger)
        self.undock = rospy.ServiceProxy("/spot/undock", Trigger)
        self.stop = rospy.ServiceProxy("/spot/stop", Trigger)
        self.power_off = rospy.ServiceProxy("/spot/power_off", Trigger)
        self.release = rospy.ServiceProxy("/spot/release", Trigger)
        self.robot_mover = actionlib.SimpleActionClient('/spot/trajectory', TrajectoryAction)
        self.robot_mover.wait_for_server()
        rospy.loginfo("[SPOT_CONTROL]: Done Initializing!")

    def control_update_cb(self, msg):
        resp = TriggerResponse
        if msg.data == "claim":
            resp = self.claim()
        elif msg.data == "power on":
            resp = self.power_on(TriggerRequest())
        elif msg.data == "dock":
            resp = self.dock(TriggerRequest())
        elif msg.data == "undock":
            resp = self.undock(TriggerRequest())
        elif msg.data == "stop":
            resp = self.stop(TriggerRequest())
        elif msg.data == "power off":
            resp = self.power_off(TriggerRequest())
        elif msg.data == "release":
            resp = self.release(TriggerRequest())
        else:
            rospy.logerr("[SPOT_CONTROL]: Received an unknown control string")
            return
        if not resp.success:
            rospy.logerr("[SPOT_CONTROL]: " + resp.message)
        else:
            rospy.loginfo("[SPOT_CONTROL]: " + resp.message + "with command " + msg.data)
        self.result_pub.publish(resp)


def main():
    rospy.init_node("spot-control-node")
    SpotControl()
    rospy.spin()


if __name__ == "__main__":
    main()

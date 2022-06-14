#! /usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String


class SpotControl:
    def __init__(self):
        rospy.Subscriber("/spot_control", String, self.control_update_cb)
        self.result_pub = rospy.Publisher("/spot_control_result", TriggerResponse, queue_size=1)
        rospy.wait_for_service("claim")
        rospy.wait_for_service("power_on")
        rospy.wait_for_service("sit")
        rospy.wait_for_service("stand")
        rospy.wait_for_service("stop")
        rospy.wait_for_service("power_off")
        rospy.wait_for_service("release")
        self.claim = rospy.ServiceProxy("claim", Trigger)
        self.power_on = rospy.ServiceProxy("power_on", Trigger)
        self.sit = rospy.ServiceProxy("sit", Trigger)
        self.stand = rospy.ServiceProxy("stand", Trigger)
        self.stop = rospy.ServiceProxy("stop", Trigger)
        self.power_off = rospy.ServiceProxy("power_off", Trigger)
        self.release = rospy.ServiceProxy("release", Trigger())

    def control_update_cb(self, msg):
        resp = TriggerResponse
        if msg.data == "claim":
            resp = self.claim(Trigger())
        elif msg.data == "power on":
            resp = self.power_on(Trigger())
        elif msg.data == "sit":
            resp = self.sit(Trigger())
        elif msg.data == "stand":
            resp = self.stand(Trigger())
        elif msg.data == "stop":
            resp = self.stop(Trigger())
        elif msg.data == "power off":
            resp = self.power_off(Trigger())
        elif msg.data == "release":
            resp = self.release(Trigger())
        else:
            rospy.logerr("[SPOT_CONTROL]: Received an unknown control string")
        if not resp.success:
            rospy.logerr("[SPOT_CONTROL]: " + resp.message)
        else:
            rospy.loginfo("[SPOT_CONTROL]: " + resp.message)
        self.result_pub.publish(resp)


def main():
    rospy.init_node("spot-control-node")
    SpotControl()
    rospy.spin()


if __name__ == "__main__":
    main()

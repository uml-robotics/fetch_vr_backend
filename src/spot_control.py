#! /usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
class SpotControl():
    def __init__(self):
        spot_control_sub = rospy.Subscriber("/spot_control", self.control_update_cb)
        rospy.wait_for_service("claim")
        rospy.wait_for_service("power_on")
        rospy.wait_for_service("sit")
        rospy.wait_for_service("stand")
        rospy.wait_for_service("stop")
        self.claim = rospy.ServiceProxy("claim", Trigger)
        self.power_on = rospy.ServiceProxy("power_on", Trigger)
        self.sit = rospy.ServiceProxy("sit", Trigger)
        self.stand = rospy.ServiceProxy("stand", Trigger)
        self.stop = rospy.ServiceProxy("stop", Trigger)

    def control_update_cb(self, msg):
        if msg.data == "claim":
            self.claim(Trigger())
        elif msg.data == "power on":
            self.power_on(Trigger())
        elif msg.data == "sit":
            self.sit(Trigger())
        elif msg.data == "stand":
            self.stand(Trigger())
        elif msg.data == "stop":
            self.stop(Trigger())
        else:
            rospy.logerr("[SPOT_CONTROL]: Recieved an unkown control string")

def main():
    rospy.init_node("spot-control-node")
    sc = SpotControl()
    rospy.spin()


if __name__ == "__main__":
    main()

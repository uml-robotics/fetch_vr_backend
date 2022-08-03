#! /usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from std_msgs.msg import String, Bool, Int32
import actionlib
from spot_msgs.msg import TrajectoryAction, TrajectoryGoal, TrajectoryResult, PowerState, ManipulatorState
from geometry_msgs.msg import PoseStamped


class SpotControl:
    def __init__(self):
        # Robot state subscribers
        rospy.Subscriber("/spot/status/power_state", PowerState, self.power_cb)
        rospy.Subscriber("/spot/status/manipulator_state", ManipulatorState, self.manipulator_cb)
        # Interactions with the actual VR UI
        self.UIState = 2 # 1 = Powered On UI, 2 = Powered Off UI
        self.ArmUIState = 1 # 1 = Stowed, 2 = Deployed
        rospy.Subscriber("/spot_control", String, self.control_update_cb)
        rospy.Subscriber("/spot_vr/trigger_publish_ui_state", Bool, self.ui_state_cb)
        rospy.Subscriber("/spot_vr/trigger_publish_arm_ui_state", Bool, self.arm_state_cb)
        self.ui_state_pub = rospy.Publisher("/spot_ui_state", Int32, queue_size=10)
        self.arm_state_pub = rospy.Publisher("/spot_arm_state", Int32, queue_size=10)
        self.result_pub = rospy.Publisher("/spot_control_result", Bool, queue_size=10)
        self.error_message_pub = rospy.Publisher("/spot_control_error_message", String, queue_size=10)
        self.nominal_body_pub = rospy.Publisher("/nominal_body_update", Bool, queue_size=10)

        # Services for controlling spot
        rospy.wait_for_service("/spot/claim")
        rospy.wait_for_service("/spot/power_on")
        rospy.wait_for_service("/spot/sit")
        rospy.wait_for_service("/spot/stand")
        rospy.wait_for_service("/spot/dock")
        rospy.wait_for_service("/spot/undock")
        rospy.wait_for_service("/spot/stop")
        rospy.wait_for_service("/spot/power_off")
        rospy.wait_for_service("/spot/release")
        rospy.wait_for_service("/spot/arm_stow")
        rospy.wait_for_service("/spot/arm_unstow")
        self.claim = rospy.ServiceProxy("/spot/claim", Trigger)
        self.power_on = rospy.ServiceProxy("/spot/power_on", Trigger)
        self.dock = rospy.ServiceProxy("/spot/dock", Trigger)
        self.undock = rospy.ServiceProxy("/spot/undock", Trigger)
        self.stop = rospy.ServiceProxy("/spot/stop", Trigger)
        self.power_off = rospy.ServiceProxy("/spot/power_off", Trigger)
        self.release = rospy.ServiceProxy("/spot/release", Trigger)
        self.robot_mover = actionlib.SimpleActionClient('/spot/trajectory', TrajectoryAction)
        self.stand = rospy.ServiceProxy("/spot/stand", Trigger)
        self.sit = rospy.ServiceProxy("/spot/sit", Trigger)
        self.stow = rospy.ServiceProxy("/spot/arm_stow", Trigger)
        self.unstow = rospy.ServiceProxy("/spot/arm_unstow", Trigger)
        self.robot_mover.wait_for_server()
        rospy.loginfo("[SPOT_CONTROL]: Done Initializing!")

    def ui_state_cb(self, msg):
        if msg.data:
            self.ui_state_pub.publish(Int32(data=self.UIState))

    def arm_state_cb(self, msg):
        if msg.data:
            self.arm_state_pub.publish(Int32(data=self.ArmUIState))
    def power_cb(self, msg):
        motor_power_state = msg.motor_power_state
        # rospy.loginfo(msg)
        if motor_power_state == PowerState().STATE_ON:
            self.UIState = 1
        else:
            self.UIState = 2
    def manipulator_cb(self, msg):
        stow_state = msg.stow_state
        if stow_state == ManipulatorState().STOWSTATE_STOWED:
            self.ArmUIState = ManipulatorState().STOWSTATE_STOWED
        else:
            self.ArmUIState = 2

    def control_update_cb(self, msg):
        resp = TriggerResponse
        if msg.data == "claim":
            resp = self.claim()
        elif msg.data == "power on":
            resp = self.power_on(TriggerRequest())
        elif msg.data == "stand":
            resp = self.stand(TriggerRequest())
            t = Bool()
            t.data = True
            self.nominal_body_pub.publish(t)
        elif msg.data == "sit":
            resp = self.sit(TriggerRequest())
        elif msg.data == "dock":
            resp = self.dock(TriggerRequest())
            pass
        elif msg.data == "undock":
            resp = self.undock(TriggerRequest())
            pass
            t = Bool()
            t.data = True
            self.nominal_body_pub.publish(t)
        elif msg.data == "stop":
            resp = self.stop(TriggerRequest())
        elif msg.data == "power off":
            resp = self.power_off(TriggerRequest())
        elif msg.data == "release":
            resp = self.release(TriggerRequest())
        elif msg.data == "stow":
            resp = self.stow(TriggerRequest())
        elif msg.data == "unstow":
            resp = self.unstow(TriggerRequest())
        else:
            rospy.logerr("[SPOT_CONTROL]: Received an unknown control string")
            return
        if not resp.success:
            rospy.logerr("[SPOT_CONTROL]: " + resp.message)
            self.error_message_pub.publish(resp.message.split(':')[-1])
        else:
            rospy.loginfo("[SPOT_CONTROL]: " + resp.message + " with command " + msg.data)
        result = Bool()
        result.data = resp.success
        self.result_pub.publish(result)


def main():
    rospy.init_node("spot-control-node")
    SpotControl()
    rospy.spin()


if __name__ == "__main__":
    main()

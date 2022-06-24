#! /usr/bin/env python3

import rospy
from std_msgs.msg import Bool, String
from spot_msgs.msg import TrajectoryAction, TrajectoryGoal
from spot_msgs.srv import ListTaggedObjectsRequest, ListTaggedObjects
from spot_msgs.srv import GetObjectPose, GetObjectPoseRequest
from geometry_msgs.msg import PoseStamped
import actionlib


class Nav:
    def __init__(self):
        self.poses = []
        self.result_pub = rospy.Publisher("/spot_nav_result", Bool, queue_size=10)
        self.tagged_object_pub = rospy.Publisher("/tagged_objects", String, queue_size=10)
        self.fiducial_pose_pub = rospy.Publisher("/fiducial_poses", PoseStamped, queue_size=10)
        rospy.Subscriber("/nav_goal_pose", PoseStamped, self.confirmation_cb)
        rospy.Subscriber("/spot/vr_body_pose", PoseStamped, self.body_pose_cb)
        rospy.Subscriber("/get_tagged_objects", Bool, self.get_tagged_objects_cb)
        rospy.Subscriber("/get_fiducial_pose", String, self.get_fiducial_pose_cb)

    def tagged_objects_client(self):
        rospy.wait_for_service("list_tagged_objects")
        get_tagged_objects = rospy.ServiceProxy("list_tagged_objects", ListTaggedObjects)
        resp = get_tagged_objects(ListTaggedObjectsRequest())
        return resp.waypoint_ids

    def fiducial_pose_client(self, id):
        rospy.wait_for_service("get_object_pose")
        get_object_pose = rospy.ServiceProxy("get_object_pose", GetObjectPose)
        req = GetObjectPoseRequest()
        req.id = id
        resp = get_object_pose(req)
        return True, resp.pose if resp.success else False, resp.message

    def confirmation_cb(self, msg):
        client = actionlib.SimpleActionClient('/spot/trajectory', TrajectoryAction)
        client.wait_for_server()

        trajectory_goal = TrajectoryGoal()
        trajectory_goal.target_pose = msg
        trajectory_goal.precise_positioning = True
        trajectory_goal.duration.data = rospy.Duration(2)

        # send goal to the action server, wait for result, print result to console
        # rospy.loginfo("Aboutta hit up the client")
        client.send_goal(trajectory_goal)
        client.wait_for_result()
        result = client.get_result()
        rospy.loginfo(result)

        # Send result to unity
        self.result_pub.publish(result.success)

    def body_pose_cb(self, msg):
        client = actionlib.SimpleActionClient('/spot/trajectory', TrajectoryAction)
        client.wait_for_server()

        trajectory_goal = TrajectoryGoal()
        trajectory_goal.target_pose = msg
        trajectory_goal.precise_positioning = True
        trajectory_goal.duration.data = rospy.Duration(2)

        # send goal to the action server, wait for result, print result to console
        # rospy.loginfo("Aboutta hit up the client")
        client.send_goal(trajectory_goal)
        client.wait_for_result()
        result = client.get_result()
        rospy.loginfo(result)

    def get_tagged_objects_cb(self, msg):
        if msg.data:
            # Get the tagged objects by calling the tagged_objects server
            tagged_object_ids = self.tagged_objects_client()
            for tagged_object in tagged_object_ids:
                self.tagged_object_pub.publish(tagged_object)

    def get_fiducial_pose_cb(self, msg):
        resp = self.fiducial_pose_client(msg.data)
        if not resp[0]:
            rospy.logerr("[SPOT_NAV]: " + resp[1])
            return
        self.fiducial_pose_pub.publish(resp[1]) # resp[1] is the pose of the object
        rospy.loginfo("[SPOT_NAV]: Success getting pose of object with ID " + msg.data)


if __name__ == "__main__":
    rospy.init_node('fetch_nav')
    spotNav = Nav()
    rospy.loginfo("[SPOT_NAV]: Backend up and running!")
    rospy.spin()
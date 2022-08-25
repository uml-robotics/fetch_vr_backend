#! /usr/bin/env python3
from bosdyn.api.graph_nav import graph_nav_pb2
import geometry_msgs.msg
import std_msgs.msg
from std_msgs.msg import Bool, String, Header, Int16MultiArray, Int32
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped, Vector3, Quaternion
from sensor_msgs.msg import PointCloud2, PointField
from spot_msgs.msg import LocalizationState
from spot_msgs.msg import NavigateToAction, NavigateToGoal, NavigateToFeedback
from bosdyn.api.graph_nav import map_pb2
from bosdyn.client.math_helpers import SE3Pose
from bosdyn.client.frame_helpers import get_a_tform_b, ODOM_FRAME_NAME
import os
import numpy as np
import rospy
from tf import transformations as ts
import tf2_ros
import actionlib
import threading

def load_map(path):
    with open(os.path.join(path, "graph"), "rb") as graph_file:
        # Load the graph file and deserialize it. The graph file is a protobuf containing only the waypoints and the
        # edges between them.
        data = graph_file.read()
        current_graph = map_pb2.Graph()
        current_graph.ParseFromString(data)

        # Set up maps from waypoint ID to waypoints, edges, snapshots, etc.
        current_waypoints = {}
        current_waypoint_snapshots = {}
        current_edge_snapshots = {}
        current_anchors = {}
        current_anchored_world_objects = {}

        # Load the anchored world objects first so we can look in each waypoint snapshot as we load it.
        for anchored_world_object in current_graph.anchoring.objects:
            current_anchored_world_objects[anchored_world_object.id] = (anchored_world_object,)
        # For each waypoint, load any snapshot associated with it.
        for waypoint in current_graph.waypoints:
            current_waypoints[waypoint.id] = waypoint

            if len(waypoint.snapshot_id) == 0:
                continue
            # Load the snapshot. Note that snapshots contain all of the raw data in a waypoint and may be large.
            file_name = os.path.join(path, "waypoint_snapshots", waypoint.snapshot_id)
            if not os.path.exists(file_name):
                continue
            with open(file_name, "rb") as snapshot_file:
                waypoint_snapshot = map_pb2.WaypointSnapshot()
                waypoint_snapshot.ParseFromString(snapshot_file.read())
                current_waypoint_snapshots[waypoint_snapshot.id] = waypoint_snapshot

                for fiducial in waypoint_snapshot.objects:
                    if not fiducial.HasField("apriltag_properties"):
                        continue

                    str_id = str(fiducial.apriltag_properties.tag_id)
                    if (str_id in current_anchored_world_objects and
                            len(current_anchored_world_objects[str_id]) == 1):
                        # Replace the placeholder tuple with a tuple of (wo, waypoint, fiducial).
                        anchored_wo = current_anchored_world_objects[str_id][0]
                        current_anchored_world_objects[str_id] = (anchored_wo, waypoint, fiducial)

        # Similarly, edges have snapshot data.
        for edge in current_graph.edges:
            if len(edge.snapshot_id) == 0:
                continue
            file_name = os.path.join(path, "edge_snapshots", edge.snapshot_id)
            if not os.path.exists(file_name):
                continue
            with open(file_name, "rb") as snapshot_file:
                edge_snapshot = map_pb2.EdgeSnapshot()
                rospy.loginfo("Parsing file {}".format(file_name))
                edge_snapshot.ParseFromString(snapshot_file.read())
                current_edge_snapshots[edge_snapshot.id] = edge_snapshot
        for anchor in current_graph.anchoring.anchors:
            current_anchors[anchor.id] = anchor
        print("Loaded graph with {} waypoints, {} edges, {} anchors, and {} anchored world objects".
              format(len(current_graph.waypoints), len(current_graph.edges),
                     len(current_graph.anchoring.anchors), len(current_graph.anchoring.objects)))
        return (current_graph, current_waypoints, current_waypoint_snapshots,
                current_edge_snapshots, current_anchors, current_anchored_world_objects)


class GraphNavLoader:
    def __init__(self, path):
        self.path = path
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.br = tf2_ros.TransformBroadcaster()
        self.current_graph, self.current_waypoints, self.current_waypoint_snapshots, \
            self.current_edge_snapshots, self.current_anchors, self.current_anchored_world_objects = load_map(path)

        # For getting waypoint ids
        rospy.Subscriber("/spot_vr_trigger_list_graph", Bool, self.list_graph_cb)
        self.list_graph_pub = rospy.Publisher("/spot_vr/waypoint_ids", String, queue_size=100)

        # For getting waypoint poses
        rospy.Subscriber("/spot_vr/trigger_get_waypoint_poses", Bool, self.get_waypoint_poses_cb)
        self.waypoint_pose_pub = rospy.Publisher("/spot_vr/waypoint_poses", PoseStamped, queue_size=100)

        # For getting snapshot origins
        rospy.Subscriber("/spot_vr/trigger_get_snapshot_origins", Bool, self.get_snapshot_origins_cb)
        self.snapshot_origin_pub = rospy.Publisher("/spot_vr/snapshot_origins", PoseStamped, queue_size=100)

        # For getting snapshot points
        rospy.Subscriber("/spot_vr/trigger_get_snapshot_points", Bool, self.get_snapshot_points_cb)
        self.snapshot_points_pub = rospy.Publisher("/spot_vr/snapshot_points", PointCloud2, queue_size=100)

        # For getting edges
        rospy.Subscriber("/spot_vr/trigger_get_edges", Bool, self.get_edges_cb)
        self.edge_pub = rospy.Publisher("/spot_vr/edges", String, queue_size=1)

        # For performing localization
        self.localization_state = None
        rospy.Subscriber("/spot_vr/trigger_localization", Bool, self.localize_cb)
        rospy.Subscriber("/spot/status/localization_state", LocalizationState, self.localization_state_cb)
        self.localization_result_pub = rospy.Publisher("/spot_vr/localization_result", PoseStamped, queue_size=1)

        # For actually navigating to a given waypoint
        self.nav_client = actionlib.SimpleActionClient('/spot/navigate_to', NavigateToAction)
        self.nav_client.wait_for_server()
        rospy.Subscriber("/spot_vr/navigate_to", String, self.navigate_to_cb)

        # Navigation feedback stuff
        rospy.Subscriber("/spot/navigate_to/feedback", NavigateToFeedback, self.navigate_to_feedback_cb)
        self.nav_status_pub = rospy.Publisher("/spot_vr/navigate_to_status", Int32, queue_size=10)
        self.nav_waypoint_pub = rospy.Publisher("/spot_vr/closest_waypoint", String, queue_size=10)
        rospy.loginfo("[GraphNavLoader]: finished configuring!")

    def navigate_to_feedback_cb(self, msg):
        self.nav_status_pub.publish(Int32(data=msg.feedback.status))
        self.nav_waypoint_pub.publish(String(data=msg.feedback.waypoint_id))

    def navigate_to_cb(self, msg):
        rospy.loginfo("Navigating to waypoint {}".format(msg.data))
        self.nav_client.send_goal(NavigateToGoal(upload_path=self.path, navigate_to=msg.data,
                                                 initial_localization_fiducial=True))
        self.nav_client.wait_for_result()
        resp = self.nav_client.get_result()

    def localization_state_cb(self, msg):
        self.localization_state = msg

    def localize_cb(self, msg):
        odom_tform_seed = self.tf_buffer.lookup_transform("seed", "odom", rospy.Time())
        ps = PoseStamped()
        ps.header.frame_id = "odom"
        ps.pose.position = odom_tform_seed.transform.translation
        ps.pose.orientation = odom_tform_seed.transform.rotation
        self.localization_result_pub.publish(ps)

    def list_graph_cb(self, msg):
        # publish ids to a topic one by one
        for wp_id in self.current_anchors:
            rospy.loginfo(wp_id)
            self.list_graph_pub.publish(String(wp_id))

    def get_waypoint_poses_cb(self, msg):
        # publish waypoint poses to a topic one by one
        for waypoint in self.current_graph.waypoints:
            if waypoint.id in self.current_anchors:
                seed_tform_waypoint = SE3Pose.from_obj(
                    self.current_anchors[waypoint.id].seed_tform_waypoint)
                ps = PoseStamped()
                ps.pose = se3_to_msg(seed_tform_waypoint)
                ps.header.frame_id = waypoint.id
                self.waypoint_pose_pub.publish(ps)

    def get_snapshot_origins_cb(self, msg):
        # publish all snapshots to a topic on the backend
        rospy.loginfo("logging snapshot keys: ")
        for key in self.current_waypoint_snapshots:
            rospy.loginfo(key)
        for waypoint in self.current_graph.waypoints:
            if waypoint.id in self.current_anchors:
                snapshot = self.current_waypoint_snapshots[waypoint.snapshot_id]
                cloud = snapshot.point_cloud
                odom_tform_cloud = get_a_tform_b(cloud.source.transforms_snapshot, ODOM_FRAME_NAME,
                                                 cloud.source.frame_name_sensor)
                seed_tform_waypoint = SE3Pose.from_obj(self.current_anchors[waypoint.id].seed_tform_waypoint)
                waypoint_tform_odom = SE3Pose.from_obj(waypoint.waypoint_tform_ko)
                waypoint_tform_cloud = waypoint_tform_odom * odom_tform_cloud
                seed_tform_cloud = seed_tform_waypoint * waypoint_tform_cloud

                ps = PoseStamped()
                ps.pose = se3_to_msg(seed_tform_cloud)
                ps.header.frame_id = waypoint.snapshot_id
                self.snapshot_origin_pub.publish(ps)

    def get_snapshot_points_cb(self, msg):
        for waypoint in self.current_graph.waypoints:
            if waypoint.id in self.current_anchors:
                snapshot = self.current_waypoint_snapshots[waypoint.snapshot_id]
                cloud = snapshot.point_cloud
                point_cloud_data = np.frombuffer(cloud.data, dtype=np.float32).reshape(int(cloud.num_points), 3)
                self.snapshot_points_pub.publish(np_to_pointcloud2(point_cloud_data, parent_frame=waypoint.snapshot_id))

    def get_edges_cb(self, msg):
        for edge in self.current_graph.edges:
            if edge.id.from_waypoint in self.current_anchors and edge.id.to_waypoint in self.current_anchors:
                self.edge_pub.publish(String(data=edge.id.from_waypoint + '*' + edge.id.to_waypoint))


# adapted from https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0
def np_to_pointcloud2(points, parent_frame=None):
    ros_dtype = PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize

    data = points.astype(dtype).tobytes()

    fields = [PointField(
        name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyz')]

    header = Header(frame_id=parent_frame, stamp=rospy.Time.now())

    return PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3),
        row_step=(itemsize * 3 * points.shape[0]),
        data=data
    )


def se3_to_msg(pose):
    msg = Pose()
    msg.position.x = pose.position.x
    msg.position.y = pose.position.y
    msg.position.z = pose.position.z
    msg.orientation.x = pose.rotation.x
    msg.orientation.y = pose.rotation.y
    msg.orientation.z = pose.rotation.z
    msg.orientation.w = pose.rotation.w
    return msg


def main():
    rospy.init_node("graph_nav_vr")
    path = rospy.get_param("/spot_vr/graph_nav_filepath")
    graph_nav = GraphNavLoader(path)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        if not graph_nav.localization_state == None and graph_nav.localization_state.seed_tform_body:
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "body"
            t.child_frame_id = "seed"
            pose = graph_nav.localization_state.seed_tform_body
            t.transform.translation.x = pose.pose.position.x
            t.transform.translation.y = pose.pose.position.y
            t.transform.translation.z = pose.pose.position.z
            t.transform.rotation.x = pose.pose.orientation.x
            t.transform.rotation.y = pose.pose.orientation.y
            t.transform.rotation.z = pose.pose.orientation.z
            t.transform.rotation.w = pose.pose.orientation.w

            # invert the transform
            mat = ts.concatenate_matrices(ts.translation_matrix([t.transform.translation.x,
                                                                 t.transform.translation.y,
                                                                 t.transform.translation.z]),
                                          ts.quaternion_matrix([t.transform.rotation.x,
                                                                t.transform.rotation.y,
                                                                t.transform.rotation.z,
                                                                t.transform.rotation.w]))
            inversed_trans = ts.inverse_matrix(mat)
            t.transform.translation = Vector3(*ts.translation_from_matrix(inversed_trans))
            t.transform.rotation = Quaternion(*ts.quaternion_from_matrix(inversed_trans))
            graph_nav.br.sendTransform(t)
            rate.sleep()
    rospy.spin()


if __name__ == "__main__":
    main()

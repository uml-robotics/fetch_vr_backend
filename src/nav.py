import rospy
from std_msgs.msg import Boolean
from geometry_msgs.msg import PoseArray
poses = []

def nav_goal_cb(msg):
    # unpack the pose array, add each pose to the array poses
    pass

def confirmation_cb(msg):
    if msg.data:
        # pop front pose off of poses
        # send pose to the spot-ros handle trajectory server
        pass

def main():
    rospy.init_node('fetch-nav-backend')
    rospy.Subscriber("/nav_confirmation", Boolean, confirmation_cb)
    rospy.Subscriber("/navigation_goal", PoseArray, nav_goal_cb)

if __name__ == "__main__":
    main()
#! /usr/bin/env python2
import rospy
from std_msgs.msg import String
# import csv

# stored csv
data_csv = []

def vrEventCallback(msg):
    # do some processing on the message here (create a csv, etc.)
    pass

def main():
    rospy.init_node("vr_event_logger")
    rospy.Subscriber('/vr_event', String, vrEventCallback, queue_size=100)
    while not rospy.is_shutdown():
        rospy.sleep(1)

if __name__ == "__main__":
    main()
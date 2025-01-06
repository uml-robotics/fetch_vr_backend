#!/usr/bin/env python

import rospy
from std_msgs.msg import String

pub = rospy.Publisher('/robot_description', String, queue_size = 10)

rospy.init_node('description_republisher')

r = rospy.Rate(1)

while not rospy.is_shutdown():
    description = rospy.get_param("/robot_description")
    pub.publish(description)
    r.sleep()

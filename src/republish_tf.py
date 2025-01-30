#!/usr/bin/env python
import rospy
from tf2_msgs.msg import TFMessage

Data = TFMessage
#Data.transforms = 

def callback(data):
    global Data
    now = rospy.get_rostime()
    rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
#    print (data)
    for d in data.transforms:
        d.header.stamp = now
        print d
#        Data.transforms.append(d)
    Data = data

def listener():
    global Data
    rospy.init_node('tf_repub')
    pub = rospy.Publisher("/tf", TFMessage)
    rospy.Subscriber("/tf_static", TFMessage, callback)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(Data)
        rate.sleep()

if __name__ == '__main__':
    listener()

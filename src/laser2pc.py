#!/usr/bin/env python

#!/usr/bin/env python
import roslib; roslib.load_manifest('laser_assembler')
import rospy
from laser_assembler.srv import *
from sensor_msgs.msg import PointCloud2


rospy.init_node("laser_to_cloud")
rospy.wait_for_service("assemble_scans2")
pub = rospy.Publisher ("/base_scan_cloud", PointCloud2, queue_size=1)

rate = rospy.Rate(10) # 10hz

while not rospy.is_shutdown():
    try:
        assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
        resp = assemble_scans(rospy.Time(0,0), rospy.get_rostime())
        pub.publish(resp.cloud)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    rate.sleep()

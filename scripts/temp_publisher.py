# publish to the topic /front/scan type LaserScan

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
import math

scan=LaserScan()

if __name__=="__main__":
    rospy.init_node("dummy_publisher")
    pub = rospy.Publisher("/front/scan", LaserScan, queue_size=10)
    rate = rospy.Rate(20)
    scan.header.frame_id = "base_link"
    scan.angle_min = -2.3561899662017822
    scan.angle_max = +2.3561899662017822
    scan.angle_increment = 0.006544972128338284
    for i in range(0,720):
        scan.ranges.append(1)
    while not rospy.is_shutdown():
        pub.publish(scan)
        rate.sleep()
# to determine the nearest objects vector
import rospy
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
import math

scan=LaserScan()

def callback(data: LaserScan):
    global scan
    scan = data

if __name__=="__main__":
    rospy.init_node("lidar_msgs")
    
    # get the rosparam tune
    tune = float(rospy.get_param("/tune"))
    rospy.Subscriber("/front/scan", LaserScan, callback)
    rate = rospy.Rate(20)
    rospy.wait_for_message("/front/scan",LaserScan)
    
    #scan information example
    # print(len(scan.ranges), (scan.angle_max-scan.angle_min)/scan.angle_increment)
    # print(type(scan.ranges))


    # define body field over the lidar, if the distance is less than r, then collision is iminent
    side = 0.42
    r = (math.sqrt(2)*side)/2
    effective_radius = (1 + tune/100)*r
    rospy.loginfo(f"Effective radius set to {effective_radius}")

    # create LaserScan message as body field
    body_field = LaserScan()
    body_field.header.frame_id = "base_link"
    body_field.angle_min = scan.angle_min
    body_field.angle_max = scan.angle_max
    body_field.angle_increment = scan.angle_increment
    body_field.range_min = scan.range_min
    body_field.range_max = scan.range_max
    body_field.ranges = [effective_radius for i in range(len(scan.ranges))]

    # while not rospy.is_shutdown():
    #     # get for closest angle
    #     # print(f"angel max :{scan.angle_max},angel min : {scan.angle_min},angle increment : {scan.angle_increment}")
    #     # print(f"range_max : {scan.range_max}, range_min : {scan.range_min}")


    #     rate.sleep()
# to determine the nearest objects vector
import rospy
import sys, threading
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
import math

scan=LaserScan()

def ping_collision(i,body_field : LaserScan): # parse i*60 -> (i+1)*60
    while not rospy.is_shutdown():
        for j in range(i*60,(i+1)*60):
            if scan.ranges[j] < body_field.ranges[j]:
                rospy.logwarn(f"Collision detected in sector {i} with distance {scan.ranges[j]}")
                break


def callback(data: LaserScan):
    global scan
    scan = data

if __name__=="__main__":
    rospy.init_node("lidar_msgs")
    rospy.loginfo("Initializing the collision_detection")
    tune = float(sys.argv[1])

    rospy.Subscriber("/front/scan", LaserScan, callback)
    rate = rospy.Rate(20)
    rospy.wait_for_message("/front/scan",LaserScan)

    # define body field over the lidar, if the distance is less than r, then collision is iminent
    side = 0.42
    r = (math.sqrt(2)*side)/2
    effective_radius = (1 + tune/100)*r
    rospy.loginfo(f"Effective radius set to {effective_radius}")

    ### body declaration
    # ##############################     CURRENTLY THE BODY FIELD IS A CYLINDER, TO BE REPLACED BY A BOX ###########
    # create LaserScan message as body field
    body_field = LaserScan()
    body_field.header.frame_id = "base_link"
    body_field.angle_min = scan.angle_min
    body_field.angle_max = scan.angle_max
    body_field.angle_increment = scan.angle_increment
    body_field.range_min = scan.range_min
    body_field.range_max = scan.range_max
    body_field.ranges = [effective_radius for i in range(len(scan.ranges))]

    # create 12 threads
    threads = [threading.Thread(target=ping_collision, args=(i,body_field)) for i in range(12)]
    for thread in threads:
        thread.start()
    
# collision_detection system entirely based on lidar data
# the body field is a cylinder of radius r, centered at the base_link, Lidar is assumed to have be at the center
# parameters used : tune -> to the linearly tune the effective radius of the body field


import rospy
import sys, threading
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
import math

scan=LaserScan()

def ping_collision(i,sectors,body_field : LaserScan): # parse i*60 -> (i+1)*60
    n = int(720/sectors)
    while not rospy.is_shutdown():
        for j in range(i*n,(i+1)*n):
            if scan.ranges[j] < body_field.ranges[j]:
                rospy.logwarn(f"Collision detected in sector {i} with distance {scan.ranges[j]}")
                break


def callback(data: LaserScan):
    global scan
    scan = data

def body_field_generator(length,width,clearance,start,end,step):
    
    # to generate an array of ranges representing a box of length + clearance, width + clearance
    # start and end are the angles of the lidar
    # step is the angle increment of the lidar

    ranges = list()
    rospy.loginfo(f"{length},{width},{clearance},{start},{end},{step}")
    alpha = end
    theta = math.atan(width/length)
    X_intercept = length/2 + clearance
    Y_intercept = width/2 + clearance
    for i in range(0,720):
        if (alpha >= start and alpha <= -(math.pi - theta) )or (alpha >= (math.pi - theta) and alpha <= end):
            y = -X_intercept/math.tan(alpha)
            x = -X_intercept
        elif (alpha >= -(math.pi - theta) and alpha <= -theta):
            x = -Y_intercept/math.tan(alpha)
            y = -Y_intercept
        elif (alpha >= -theta and alpha <= theta):
            y = X_intercept/math.tan(alpha)
            x = X_intercept
        elif (alpha >= theta and alpha <= (math.pi - theta)):
            x = Y_intercept/math.tan(alpha)
            y = Y_intercept
        else : 
            rospy.loginfo("Error in body field generation")
        ranges.append(math.sqrt(x**2 + y**2))
        alpha += step
    return ranges


    # pass

if __name__=="__main__":

    rospy.init_node("collision_detector")

    rospy.loginfo("Collision_detector started, Initializing the parameters")

    # get parameters
    tune = rospy.get_param("/collision_detection/tune")
    sectors = rospy.get_param("/collision_detection/sectors")
    length = rospy.get_param("/robot_size/length")
    width = rospy.get_param("/robot_size/width")
    clearance = rospy.get_param("/collision_detection/clearance")

    rospy.loginfo(f"Tune = {tune}")
    rospy.loginfo(f"Sectors = {sectors}")
    rospy.loginfo(f"Length = {length}")
    rospy.loginfo(f"width = {width}")
    rospy.loginfo(f"clearance = {clearance}")

    rospy.Subscriber("/front/scan", LaserScan, callback)
    rate = rospy.Rate(20)
    rospy.wait_for_message("/front/scan",LaserScan)

    # define body field over the lidar, if the distance is less than r, then collision is iminent
    side = max(length,width)
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

    body_field.ranges = body_field_generator(length, width, clearance, scan.angle_min, scan.angle_max, scan.angle_increment)
    print(f"{len(body_field.ranges)}")
    # ## body declaration to be changed to a box
    # body_field.ranges = [effective_radius for i in range(len(scan.ranges))]

    # # Create N threads to check for collision in N sectors
    # threads = [threading.Thread(target=ping_collision, args=(i,sectors,body_field,)) for i in range(12)]
    # for thread in threads:
    #     thread.start()
    

    
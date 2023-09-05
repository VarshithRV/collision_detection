# subscribes to /keyboard and publishes to cmd_vel
import rospy
from collision_detection.msg import key
from geometry_msgs.msg import Twist
key_values = key()

def keycb(data: key):
    global key_values
    key_values = data

if __name__=="__main__":
    rospy.init_node("keyboard_control")
    rospy.Subscriber("/keyboard", key, keycb)
    rate = rospy.Rate(20)
    pub_vel=rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    velocity=Twist()

    rospy.loginfo("Keyboard control node started, start controlling using keyboard")

    while not rospy.is_shutdown():
        velocity.linear.x = key_values.w-key_values.s
        velocity.angular.z = key_values.a-key_values.d
        pub_vel.publish(velocity)
        rate.sleep()
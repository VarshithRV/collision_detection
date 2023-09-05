# key publisher

# publishes the arrow key values to /keyboard/arrow
# can detect multiple keys at a time
# get a dictionary that contains the current state of the up, down, side and left keys

import rospy
from pynput.keyboard import Key, Listener
import subprocess
import sys
import threading
from collision_detection.msg import key
# from px4_controller.msg import key
from std_msgs.msg import String, Header

key_values = key()

key_state={
    "key_up":0,
    "key_down":0,
    "key_left":0,
    "key_right":0,
    "key_left_shift":0,
    "key_left_ctrl":0,
    "w":0,
    "a":0,
    "s":0,
    "d":0,
    "q":0,
    "e":0,
    "c":0,
    "z":0,
    "x":0,
    "j":0
}

def assign():
    rate = rospy.Rate(20)
    while True:
        key_values.key_up=key_state["key_up"]
        key_values.key_down=key_state["key_down"]
        key_values.key_left=key_state["key_left"]
        key_values.key_right=key_state["key_right"]
        key_values.key_left_shift=key_state["key_left_shift"]
        key_values.key_left_ctrl=key_state["key_left_ctrl"]
        key_values.w=key_state['w']
        key_values.a=key_state['a']
        key_values.s=key_state['s']
        key_values.d=key_state['d']
        key_values.q=key_state['q']
        key_values.e=key_state['e']
        key_values.c=key_state['c']
        key_values.z=key_state['z']
        key_values.x=key_state['x']
        key_values.j=key_state['j']
        pub.publish(key_values)
        rate.sleep()

def on(key:Key):
    key = str(key)
    # print(key)
    # if(key=="Key.esc"):
    #     print("Exiting")
    #     subprocess.run(["stty","echo"]) # password mode
    #     sys.exit() # to the parent thread
    if(key=="Key.up"):
        key_state["key_up"]=1
    if(key=="Key.down"):
        key_state["key_down"]=1
    if(key=="Key.left"):
        key_state["key_left"]=1
    if(key=="Key.right"):
        key_state["key_right"]=1
    if(key=="Key.shift"):
        key_state["key_left_shift"]=1
    if(key=="Key.ctrl"):
        key_state["key_left_ctrl"]=1
    if(key=="'w'"):
        key_state["w"]=1
    if(key=="'a'"):
        key_state["a"]=1
    if(key=="'s'"):
        key_state["s"]=1
    if(key=="'d'"):
        key_state["d"]=1
    if(key=="'q'"):
        key_state["q"]=1
    if(key=="'e'"):
        key_state["e"]=1
    if(key=="'c'"):
        key_state["c"]=1
    if(key=="'z'"):
        key_state["z"]=1
    if(key=="'x'"):
        key_state["x"]=1
    if(key=="'j'"):
        key_state["j"]=1

def off(key:Key):
    key = str(key)
    if(key=="Key.up"):
        key_state["key_up"]=0
    if(key=="Key.down"):
        key_state["key_down"]=0
    if(key=="Key.left"):
        key_state["key_left"]=0
    if(key=="Key.right"):
        key_state["key_right"]=0
    if(key=="Key.shift"):
        key_state["key_left_shift"]=0
    if(key=="Key.ctrl"):
        key_state["key_left_ctrl"]=0
    if(key=="'w'"):
        key_state["w"]=0
    if(key=="'a'"):
        key_state["a"]=0
    if(key=="'s'"):
        key_state["s"]=0
    if(key=="'d'"):
        key_state["d"]=0
    if(key=="'q'"):
        key_state["q"]=0
    if(key=="'e'"):
        key_state["e"]=0
    if(key=="'c'"):
        key_state["c"]=0
    if(key=="'z'"):
        key_state["z"]=0
    if(key=="'x'"):
        key_state["x"]=0
    if(key=="'j'"):
        key_state["j"]=0

rospy.init_node("Keypub")  
pub=rospy.Publisher("/keyboard",key,queue_size=10)
print("Enter the key here, use ctrl c to exit,press esc to exit")
subprocess.run(["stty","-echo"])
listener = Listener(on_press=on,on_release=off)
listener.start()
thread1=threading.Thread(target=assign)
thread1.start()
thread1.join()
listener.join()



#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import *

"""
def callback(data):
    print(data.pose.position)
    #rospy.loginfo(rospy.get_caller_id() + "I hello : %s ", data)
    
def listener():
    # Initialise listener node
    rospy.init_node('listener', anonymous=True)
    # Take the first message from a topic and unsubscribe
    message = rospy.wait_for_message("tool_pose", PoseStamped, 10)

    # Use this for subscribing to a topic
    #rospy.Subscriber("tool_pose", PoseStamped, callback)

    #print(message.pose)
    x = message.pose.orientation.x
    y = message.pose.orientation.y
    z = message.pose.orientation.z
    w = message.pose.orientation.w

    euler_angles = euler_from_quaternion([w, x, y, z])
    #print(euler_angles)
"""

def get_position():
    rospy.init_node('listener', anonymous=True)
    #rospy.Subscriber("tool_pose", PoseStamped, callback)
    message = rospy.wait_for_message("tool_pose", PoseStamped, 10)

    x = message.pose.position.x
    y = message.pose.position.y
    z = message.pose.position.z

    #print(message.pose.orientation)
    ex = message.pose.orientation.x
    ey = message.pose.orientation.y
    ez = message.pose.orientation.z
    ew = message.pose.orientation.w

    euler_angles = euler_from_quaternion([ex, ey, ez, ew])
    euler_pose = [x, y, z] + list(euler_angles)
    rounded_pose = [round(num, 5) for num in euler_pose]
    print(rounded_pose)
    
 

if __name__ == '__main__':
    get_position()
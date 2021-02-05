#!/usr/bin/env python

import rospy
import tf2_ros
import tf_conversions
from tm_msgs.msg import *
from tm_msgs.srv import *
from geometry_msgs.msg import PoseStamped
from tf.transformations import *


def get_position():
    message = rospy.wait_for_message("tool_pose", PoseStamped, 5)

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
    rounded_pose = [round(num, 2) for num in euler_pose]
    return rounded_pose


def replace(data, z_value):
    newlist = data[:]
    newlist[2] = z_value
    return newlist

def parser():
    #pick = raw_input("Enter the pick coordinates:")
    #place = raw_input("Enter the place coordinates:")
    #pick = map(float, pick.split(','))
    #place = map(float, place.split(','))
    pick = [0.1, -0.5, 0.3, 3.14159, 0.0, 0.0]
    place = [-0.4, -0.5, 0.2, 3.14159, 1.57, 0.0]
    z_value = max(pick[2], place[2]) + 0.1
    z_pick = replace(pick, z_value)
    z_place = replace(place, z_value)
    return pick, z_pick, place, z_place


def test_move():
    home_position = [0,-0.5,0.5,3.14159,0,0]

    # Initialise node
    rospy.init_node('test_move')

    # Using services
    rospy.wait_for_service('tm_driver/set_event')
    rospy.wait_for_service('tm_driver/set_positions')
    rospy.wait_for_service('tm_driver/ask_sta')

    # rospy.ServiceProxy(<Service>, <Service definition (.srv file)>)
    set_event = rospy.ServiceProxy('tm_driver/set_event', SetEvent)
    set_positions = rospy.ServiceProxy('tm_driver/set_positions', SetPositions)
    ask_sta = rospy.ServiceProxy('tm_driver/ask_sta', AskSta)


if __name__ == '__main__':
    try:
        test_move()
    except rospy.ROSInterruptException:
        pass
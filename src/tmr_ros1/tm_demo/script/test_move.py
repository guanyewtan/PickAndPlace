#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_msgs
import tf_conversions
import tm_msgs.msg
from tm_msgs.srv import *
from geometry_msgs.msg import *
from tf.transformations import *


def get_position():
    message = rospy.wait_for_message("tool_pose", PoseStamped, 5)

    x = message.pose.position.x
    y = message.pose.position.y
    z = message.pose.position.z

    ex = message.pose.orientation.x
    ey = message.pose.orientation.y
    ez = message.pose.orientation.z
    ew = message.pose.orientation.w

    euler_angles = euler_from_quaternion([ex, ey, ez, ew])
    euler_pose = [x, y, z] + list(euler_angles)
    rounded_pose = [round(num, 2) for num in euler_pose]
    return rounded_pose

def get_translate(trans):
    x = trans.transform.translation.x
    y = trans.transform.translation.y
    z = trans.transform.translation.z

    #print(message.pose.orientation)
    ex = trans.transform.rotation.x
    ey = trans.transform.rotation.y
    ez = trans.transform.rotation.z
    ew = trans.transform.rotation.w

    euler_angles = euler_from_quaternion([ex, ey, ez, ew])
    euler_pose = [x, y, z] + list(euler_angles)
    rounded_pose = [round(num, 2) for num in euler_pose]
    return rounded_pose

def handle_io(state):
    set_event(SetEventRequest.TAG, 1, 0)
    #print("tag set!", i)
    while True:
        rospy.sleep(0.2)
        res = ask_sta('01', '1', 1)
        #print(i, res)
        if res.subcmd == '01':
            data = res.subdata.split(',')
            if data[1] == 'true':
                if data[0] == '1':
                    if state == 'pick':
                        rospy.loginfo('Pick position reached!')
                        set_io(SetIORequest.MODULE_ENDEFFECTOR, 1, 0, 1)
                        rospy.sleep(0.2)
                        set_io(SetIORequest.MODULE_ENDEFFECTOR, 1, 0, 0)
                        rospy.sleep(1.5)
                        break
                    if state == 'place':
                        rospy.loginfo('Place position reached!')
                        set_io(SetIORequest.MODULE_ENDEFFECTOR, 1, 0, 1)
                        set_io(SetIORequest.MODULE_ENDEFFECTOR, 1, 1, 1)
                        rospy.sleep(0.5)
                        set_io(SetIORequest.MODULE_ENDEFFECTOR, 1, 0, 0)
                        set_io(SetIORequest.MODULE_ENDEFFECTOR, 1, 1, 0)
                        #rospy.sleep(1)
                        break

def replace(data, z_value):
    newlist = data[:]
    newlist[2] += z_value
    return newlist

def test_move():
    home_position = [0,-0.5,0.5,-3.14159,0,0]

    # Read the pick/place/safe positions
    print("Start reading...")
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    trans_pick = tfBuffer.lookup_transform('base', 'pick', rospy.Time(), rospy.Duration(1.0))
    trans_safepick = tfBuffer.lookup_transform('base', 'safe_pick', rospy.Time(), rospy.Duration(1.0))
    trans_place = tfBuffer.lookup_transform('base', 'place', rospy.Time(), rospy.Duration(1.0))
    trans_safeplace = tfBuffer.lookup_transform('base', 'safe_place', rospy.Time(), rospy.Duration(1.0))


    # Set the positions
    set_positions(SetPositionsRequest.PTP_T, home_position, 4, 0.4, 0, False)
    set_positions(SetPositionsRequest.PTP_T, get_translate(trans_safepick), 4, 0.4, 0, False)
    set_positions(SetPositionsRequest.PTP_T, get_translate(trans_pick), 4, 0.4, 0, False)
    handle_io("pick")
    set_positions(SetPositionsRequest.PTP_T, get_translate(trans_safepick), 4, 0.4, 0, False)
    set_positions(SetPositionsRequest.PTP_T, get_translate(trans_safeplace), 4, 0.4, 0, False)
    set_positions(SetPositionsRequest.PTP_T, get_translate(trans_place), 4, 0.4, 0, False)
    handle_io("place")
    set_positions(SetPositionsRequest.PTP_T, get_translate(trans_safeplace), 4, 0.4, 0, False)
    set_positions(SetPositionsRequest.PTP_T, replace(get_translate(trans_safeplace), 0.3), 4, 0.4, 0, False)
    set_positions(SetPositionsRequest.PTP_T, home_position, 4, 0.4, 0, False)


if __name__ == '__main__':

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
    set_io = rospy.ServiceProxy('tm_driver/set_io', SetIO)

    try:
        test_move()
    except rospy.ROSInterruptException:
        pass
"""
    rosservice call /tm_driver/set_io 1 1 0 1
    rosservice call /tm_driver/set_io 1 1 1 1
    rosservice call /tm_driver/set_io 1 1 0 0
    rosservice call /tm_driver/set_io 1 1 1 0

    test
"""
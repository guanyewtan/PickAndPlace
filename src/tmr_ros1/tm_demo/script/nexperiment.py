#!/usr/bin/env python  
import rospy
import tf2_ros
import tf2_msgs
import tf_conversions
import tm_msgs.msg
from tm_msgs.srv import *
from geometry_msgs.msg import *
from tf.transformations import *

#pick = raw_input("Enter the pick coordinates:")
#place = raw_input("Enter the place coordinates:")
#pick = map(float, pick.split(','))
#place = map(float, place.split(','))
pick = [0.07, -0.5, 0.2, -3.14159, 0, 0]
#place = [-0.4, -0.5, 0.2, 3.14159, 1.57, 0.0]
place = [-0.9,0.1,0.2,-3.14159,1.57,0.0]
safe = [0.0, 0.0, -0.1, 0.0, 0.0, 0.0]

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

if __name__ == '__main__':
    rospy.init_node('test_move')

    # Using services
    rospy.wait_for_service('tm_driver/set_event')
    rospy.wait_for_service('tm_driver/set_positions')
    rospy.wait_for_service('tm_driver/ask_sta')

    set_positions = rospy.ServiceProxy('tm_driver/set_positions', SetPositions)
    ask_sta = rospy.ServiceProxy('tm_driver/ask_sta', AskSta)
    set_io = rospy.ServiceProxy('tm_driver/set_io', SetIO)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    #tfBuffer.can_transform('base', 'safe_pick', rospy.Time(0), rospy.Duration(5.0))
    trans = tfBuffer.lookup_transform('base', 'safe_pick', rospy.Time(0), rospy.Duration(5.0))
    set_positions(SetPositionsRequest.PTP_T, get_translate(trans), 4, 0.4, 0, False)
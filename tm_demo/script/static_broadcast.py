#!/usr/bin/env python
# to get commandline arguments
import sys

import geometry_msgs.msg
import rospy
# because of transformations
import tf
import tf2_ros


class staticBroadcaster:

    def __init__(self, parent, child, coord):
        self.static_transformStamped = geometry_msgs.msg.TransformStamped()

        self.static_transformStamped.header.stamp = rospy.Time.now()
        self.static_transformStamped.header.frame_id = parent
        self.static_transformStamped.child_frame_id = child

        self.static_transformStamped.transform.translation.x = coord[0]
        self.static_transformStamped.transform.translation.y = coord[1]
        self.static_transformStamped.transform.translation.z = coord[2]

        quat = tf.transformations.quaternion_from_euler(coord[3], coord[4], coord[5])
        self.static_transformStamped.transform.rotation.x = quat[0]
        self.static_transformStamped.transform.rotation.y = quat[1]
        self.static_transformStamped.transform.rotation.z = quat[2]
        self.static_transformStamped.transform.rotation.w = quat[3]

        



if __name__ == '__main__':
    pick = [0.07, -0.5, 0.2, -3.14159, 0, 0]
    place = [-0.9,0.1,0.2,-3.14159,1.57,0]

    rospy.init_node('static_pickplace_broadcaster')
    #broadcaster = tf2_ros.StaticTransformBroadcaster()
    #broadcaster2 = tf2_ros.StaticTransformBroadcaster()
    #pick_broadcast = staticBroadcaster("base", "pick", pick)
    #place_broadcast = staticBroadcaster("base", "place", place)

    #broadcaster.sendTransform(place_broadcast.static_transformStamped)
    #broadcaster2.sendTransform(pick_broadcast.static_transformStamped)
    print("HELLLPPPPPPP\n\n\n\n\n\n\n\n")
    pick = rospy.get_param("~param_test")
    print(pick)
    
    rospy.spin()

#!/usr/bin/env python  
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
from tf.transformations import *

#pick = raw_input("Enter the pick coordinates:")
#place = raw_input("Enter the place coordinates:")
#pick = map(float, pick.split(','))
#place = map(float, place.split(','))
pick = [0.07, -0.5, 0.2, -3.14159, 0, 0]
#place = [-0.4, -0.5, 0.2, 3.14159, 1.57, 0.0]
place = [-0.9,0.1,0.2,-3.14159,1.57,0.0]
safe = [0.0, 0.0, -0.1, 0.0, 0.0, 0.0]


class FixedTFBroadcaster:
    
    def __init__(self, pick, place, safe):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        tf_pick = self.transformer(pick, "base", "pick")
        tf_place = self.transformer(place, "base", "place")
        tf_safe_pick = self.transformer(safe, "pick", "safe_pick")
        tf_safe_place = self.transformer(safe, "place", "safe_place")

        while not rospy.is_shutdown():
            # Run this loop at X Hz
            rospy.sleep(0.04)
            self.pub_tf.publish(tf_pick)
            self.pub_tf.publish(tf_place)
            #self.pub_tf.publish(tf_safe_pick)
            #self.pub_tf.publish(tf_safe_place)
    
    def transformer(self, coord, frame_parent, frame_child):
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = frame_parent
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = frame_child
        t.transform.translation.x = coord[0]
        t.transform.translation.y = coord[1]
        t.transform.translation.z = coord[2]
        rot_coord = list(quaternion_from_euler(coord[3], coord[4], coord[5]))
        t.transform.rotation.x = rot_coord[0]
        t.transform.rotation.y = rot_coord[1]
        t.transform.rotation.z = rot_coord[2]
        t.transform.rotation.w = rot_coord[3]
        tfm = tf2_msgs.msg.TFMessage([t])
        return tfm



if __name__ == '__main__':
    rospy.init_node('experiment')
    pick = [0.07, -0.5, 0.2, -3.14159, 0, 0]
    place = [-0.9,0.1,0.2,-3.14159,1.57,0]
    safe = [0.0, 0.0, -0.1, 0.0, 0.0, 0.0]
    pick_broadcast = FixedTFBroadcaster(pick, place, safe)

    rospy.spin()
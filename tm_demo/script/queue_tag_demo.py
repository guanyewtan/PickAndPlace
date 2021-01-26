#!/usr/bin/env python

import rospy
from tm_msgs.msg import *
from tm_msgs.srv import *


def callback(msg):
    rospy.loginfo(rospy.get_caller_id() + ': %s', msg.subdata)
    if msg.subcmd == '01':
        data = msg.subdata.split(',')
        if data[1] == 'true':
            rospy.loginfo('point (Tag %s) is reached', data[0])

def queue_tag_demo(by_polling):
    rospy.init_node('queue_tag_demo')

    if not by_polling:
        # listen to 'tm_driver/sta_response' topic
        rospy.Subscriber('tm_driver/sta_response', StaResponse, callback)
        #return(0)

    # using services
    rospy.wait_for_service('tm_driver/set_event')
    rospy.wait_for_service('tm_driver/set_positions')
    rospy.wait_for_service('tm_driver/ask_sta')

    #rospy.ServiceProxy(<Service>, <Service definition (.srv file)>)
    set_event = rospy.ServiceProxy('tm_driver/set_event', SetEvent)
    set_positions = rospy.ServiceProxy('tm_driver/set_positions', SetPositions)
    ask_sta = rospy.ServiceProxy('tm_driver/ask_sta', AskSta)

    # 4 points (joint angle[rad])
    points = [
        [0.,0.,0.,0.,0.,0.],
        [0.,0.4,0.,0.,0.],
        [0.,0.,0.8,0.,0.,0.],
        [0.,0.,1.2,0.,0.,0.]
    ]

    # send 4 motion command
    # and set QueueTag command right after motion command (Tag: 1 -> 2 -> 3 -> 4)
    for i in range(4):
        set_positions(SetPositionsRequest.PTP_J, points[i], 3, 0.4, 0, False)
        set_event(SetEventRequest.TAG, i + 1, 0)

    if by_polling:
        # ask sta to check QueueTag state
        i = 0
        while i < 4:
            rospy.sleep(0.2)
            res = ask_sta('01', str(i + 1), 1)
            print(res)
            if res.subcmd == '01':
                data = res.subdata.split(',')
                if data[1] == 'true':
                    rospy.loginfo('point %d (Tag %s) is reached', i + 1, data[0])
                    i = i + 1
    else:
        # spin() keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    try:
        queue_tag_demo(False)
        #queue_tag_demo(True)
    except rospy.ROSInterruptException:
        pass

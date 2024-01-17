#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size = 10)
    rospy.init_node('talker', anonymous = False)
    rate = rospy.Rate(10)
    cnt = 0
    while not rospy.is_shutdown():
        chat_str = "Hello World, this is my first ROS node."
        chat_str = chat_str + "This message published %s times" % cnt
        cnt = cnt+1
        pub.publish(chat_str)
        rate.sleep()

if __name__== '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    
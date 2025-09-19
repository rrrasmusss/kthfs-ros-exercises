#!/usr/bin/env python
# Code inspired from ROS Wiki
# and https://www.youtube.com/watch?v=otGWUZqB9XE

#nodeA
import rospy
from std_msgs.msg import Int16

def number_publisher():
    rospy.init_node('nodeA')
    pub = rospy.Publisher('/Nordahl', Int16, queue_size=1)
    rate = rospy.Rate(20) # 20hz
    out_msg = Int16()

    n = 4
    k = n  # ensure k > 0

    while not rospy.is_shutdown():
        out_msg.data = k
        pub.publish(out_msg)
        k += n
        rate.sleep()

if __name__ == '__main__':
    try:
        number_publisher()
    except rospy.ROSInterruptException:
        pass
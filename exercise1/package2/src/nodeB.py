#!/usr/bin/env python
# Code inspired from ROS Wiki
# and https://www.youtube.com/watch?v=otGWUZqB9XE

#nodeB
import rospy
from std_msgs.msg import Float32, Int16

class NodeB:
    def __init__(self):
        self.pub = rospy.Publisher('/kthfs/result', Float32, queue_size=1)
        self.number_subscriber = rospy.Subscriber('/Nordahl', Int16, self.num_CB)
        self.out_msg = Float32()
        self.q = 0.15
        
    def num_CB(self, msg):
        result_value = float(msg.data / self.q)
        self.out_msg.data = result_value
        self.pub.publish(self.out_msg)


if __name__ == '__main__':
    try:
        rospy.init_node("nodeB")
        NodeB()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
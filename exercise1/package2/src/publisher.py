# #!/usr/bin/env python
# # license removed for brevity
# #nodeA
# import rospy
# from std_msgs.msg import Float32

# def number_publisher():
#     pub = rospy.Publisher('~/kthfs/result', Float32, queue_size=10)
#     rospy.init_node('int_publisher', anonymous=True)
#     rate = rospy.Rate(20) # 20hz

#     while not rospy.is_shutdown():

#         pub.publish()
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         number_publisher()
#     except rospy.ROSInterruptException:
#         pass
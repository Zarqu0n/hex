#! /usr/bin/env python
from joint_publisher import JointPub
import rospy
import time


leg6_pos1 = [0.0,0.3,-3.27]
leg6_pos2 = [3.14,1,6.57]
leg6_rate_value = 0.5

if __name__ == '__main__':
    while not rospy.is_shutdown():
        leg6 = JointPub("6")
        leg6.move(leg6_pos1)
        time.sleep(1)
        leg6.move(leg6_pos2)

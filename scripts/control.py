#! /usr/bin/env python
from joint_publisher import JointPub
import rospy
import time

leg1_pos1 = [0.0,0.3,-3.27]
leg1_pos2 = [0.0,3,-1.57]
leg1_rate_value = 0.5

leg2_pos1 = [0.0,0.3,-3.27]
leg2_pos2 = [1.0,2,1.57]
leg2_rate_value = 0.5

leg3_pos1 = [0.0,0.3,-3.27]
leg3_pos2 = [4.5,2,3.57]
leg3_rate_value = 0.5

leg4_pos1 = [0.0,0.3,-3.27]
leg4_pos2 = [5.0,1,3.27]
leg4_rate_value = 0.5

leg5_pos1 = [0.0,0.3,-3.27]
leg5_pos2 = [2.0,1.3,6.57]
leg5_rate_value = 0.5

leg6_pos1 = [0.0,0.3,-3.27]
leg6_pos2 = [8.0,1,6.57]
leg6_rate_value = 0.5

if __name__ == '__main__':
    while not rospy.is_shutdown():
        leg1 = JointPub("1")
        leg2 = JointPub("2")
        leg3 = JointPub("3")
        leg4 = JointPub("4")
        leg5 = JointPub("5")
        leg6 = JointPub("6")
        leg1.start_loop(leg1_rate_value,leg1_pos1,leg1_pos2)
        leg3.start_loop(leg2_rate_value,leg3_pos1,leg3_pos2)
        leg5.start_loop(leg2_rate_value,leg5_pos1,leg5_pos2)
        time.sleep(2)
        leg1.start_loop(leg1_rate_value,leg1_pos2,leg1_pos1)
        leg3.start_loop(leg2_rate_value,leg3_pos2,leg3_pos1)
        leg5.start_loop(leg2_rate_value,leg5_pos2,leg5_pos1)
        leg2.start_loop(leg1_rate_value,leg2_pos1,leg2_pos2)
        leg4.start_loop(leg2_rate_value,leg4_pos1,leg4_pos2)
        leg6.start_loop(leg2_rate_value,leg6_pos1,leg6_pos2)
        time.sleep(2)
        leg2.start_loop(leg1_rate_value,leg2_pos2,leg2_pos1)
        leg4.start_loop(leg2_rate_value,leg4_pos2,leg4_pos1)
        leg6.start_loop(leg2_rate_value,leg6_pos2,leg6_pos1)

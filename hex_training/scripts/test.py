#! /usr/bin/env python3
from hex_control.legCtrl import LegController
import rospy
import time

if __name__ == "__main__":
    rospy.init_node("hexapod_control_node")
    leg = LegController(leg=5)
    time.sleep(1)
    rospy.loginfo("Starting")
    print(leg.getContact())


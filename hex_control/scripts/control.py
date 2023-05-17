#! /usr/bin/env python
from joint_publisher import LegController
import rospy
import time

class HexapodController:
    def __init__(self)->None:
        self.leg2 = LegController(2)

    
    def main(self)->None:
        while not rospy.is_shutdown():
            self.leg2.set_angles([0,0,0])
            time.sleep(2)
            self.leg2.set_angles([0,3.14,-1.57])
            time.sleep(2)
            self.leg2.set_angles([1.57,0,-1.57])
            time.sleep(2)
            self.leg2.set_angles([3.14,1.57,-1.57])
            time.sleep(2)

if __name__ == "__main__":
    rospy.init_node("hexapod_controller")
    hc = HexapodController()
    hc.main()
    rospy.spin()

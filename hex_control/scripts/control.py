#! /usr/bin/env python3
from joint_publisher import LegController
import rospy
import time
from hex_msgs.srv import LegSetPose

class HexapodController:
    def __init__(self)->None:
        self.legs = [LegController(i) for i in range(6)]

        control_service_ =  rospy.Service("set_leg_pose",LegSetPose,self.set_leg_pose)

    def set_leg_pose(self,req):
        self.legs[req.legid].set_pose([req.x,req.y,req.z])
        return True
    
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
    rospy.spin()

#! /usr/bin/env python
from hex_control.legCtrl import LegController
import rospy
import time
from hex_msgs.srv import LegSetPose
from hex_msgs.msg import HexStates

class HexapodController:
    def __init__(self)->None:
    
        rospy.loginfo("Hexapod controller is starting")
        self.legs = [LegController(i) for i in range(1,7)]
        control_service_ =  rospy.Service("set_leg_pose",LegSetPose,self.set_leg_pose)

        rospy.loginfo("Services is ready")
        self.setupRobot()

    def setupRobot(self):
        self.setupLegs()
        rospy.loginfo("Legs are zero pose")
        self.setLegsAngle([0,0.52,-2.03])
        rospy.loginfo("Legs setup done")


    def set_leg_pose(self,req):
        return self.legs[req.legid].set_pose([req.x,req.y,req.z])
    
    def setupLegs(self):
        self.setLegsAngle([0,0,0])
        self.legs[0].poseCtrl()
        time.sleep(2)

        
    def setLegsAngle(self,angle_array:list=[0.0,0.0,0.0]):
        for leg in self.legs:
            leg.setAngles(angle_array)

    def setAllLegsAngle(self,angle_array:list):
        leg1_array = [angle_array[0],angle_array[5],angle_array[11]]
        leg2_array = [angle_array[1],angle_array[6],angle_array[12]]
        leg3_array = [angle_array[2],angle_array[7],angle_array[13]]
        leg4_array = [angle_array[3],angle_array[8],angle_array[14]]
        leg5_array = [angle_array[4],angle_array[9],angle_array[15]]
        leg6_array = [angle_array[5],angle_array[10],angle_array[16]]
        self.legs[0].setAngles(leg1_array)
        self.legs[1].setAngles(leg2_array)
        self.legs[2].setAngles(leg3_array)
        self.legs[3].setAngles(leg4_array)
        self.legs[4].setAngles(leg5_array)
        self.legs[5].setAngles(leg6_array)
        
if __name__ == "__main__":
    rospy.init_node("hexapod_controller")
    hc = HexapodController()
    rospy.spin()

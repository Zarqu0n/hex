#! /usr/bin/env python
import rospy
from std_msgs.msg import Float64
from inverse_kinematics import InverseKinematics as IK

class LegController():
    def __init__(self,leg):

        self.publishers_array = []
        self.base_coxa_joint_pub = rospy.Publisher('/hexapod/base_coxa_joint_{}_position_controller/command'.format(leg), Float64, queue_size=1)
        self.coxa_femur_joint_pub = rospy.Publisher('/hexapod/coxa_femur_joint_{}_position_controller/command'.format(leg), Float64, queue_size=1)
        self.femur_tibia_joint_pub = rospy.Publisher('/hexapod/femur_tibia_joint_{}_position_controller/command'.format(leg), Float64, queue_size=1)
        
        self.ik = IK()

    def set_angle(self, joint,angle:float=0):
        if joint == "coxa":
            self.base_coxa_joint_pub.publish(angle)
        elif joint == "femur":
            self.coxa_femur_joint_pub.publish(angle)
        elif joint == "tibia":
            self.femur_tibia_joint_pub.publish(angle)
        else:
            rospy.logwarn_once("Joint {} not found".format(joint))

    def set_angles(self, angle_array:list=[0,0,0]):
        self.set_angle("coxa",angle_array[0])
        self.set_angle("femur",angle_array[1])
        self.set_angle("tibia",angle_array[2])


    def set_pose(self, pose_array:list=[0,0,0]):
        angle_array = self.ik.IK(pose_array)
        rospy.loginfo(angle_array)
        self.set_angles(angle_array)
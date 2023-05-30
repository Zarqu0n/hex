#! /usr/bin/env python
import rospy
from std_msgs.msg import Float64
from hex_control.inverse_kinematics import InverseKinematics as IK
import time
from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import JointState

class LegController():
    def __init__(self,leg):

        self.leg = leg
        self.publishers_array = []

        self.base_coxa_joint_pub = rospy.Publisher('/hexapod/base_coxa_joint_{}_position_controller/command'.format(leg), Float64, queue_size=1)
        self.coxa_femur_joint_pub = rospy.Publisher('/hexapod/coxa_femur_joint_{}_position_controller/command'.format(leg), Float64, queue_size=1)
        self.femur_tibia_joint_pub = rospy.Publisher('/hexapod/femur_tibia_joint_{}_position_controller/command'.format(leg), Float64, queue_size=1)
        self.contact_sensor_sub = rospy.Subscriber('/contact_{}'.format(leg), ContactsState, self.contactSensorCb)
        self.joint_state_sub = rospy.Subscriber('/hexapod/joint_states', JointState, self.jointStateCb)
        # self.check_publishers_connection()
        self.ik = IK()
        self.base_coxa_joint_limit = [0.0, 0.0]
        self.coxa_femur_joint_limit = [0.0, 0.0]
        self.femur_tibia_joint_limit = [0.0, 0.0]
        self.contact = 0
        self.joint_state = JointState()


    def poseCtrl(self):
        time.sleep(0.2)
        return True

    def setAngle(self, joint,angle:float=0):
        if joint == "coxa":
            self.base_coxa_joint_pub.publish(angle)
        elif joint == "femur":
            self.coxa_femur_joint_pub.publish(angle)
        elif joint == "tibia":
            self.femur_tibia_joint_pub.publish(angle)
        else:
            rospy.loginfo("Joint {} not found".format(joint))

    def setAngles(self, angle_array:list=[0,0,0]):
        self.setAngle("coxa",angle_array[0])
        self.setAngle("femur",angle_array[1])
        self.setAngle("tibia",angle_array[2])

    def jointStateCb(self, msg):
        self.joint_state = msg

    def contactSensorCb(self, msg):
        """
        Callback function for the contact sensor
        :param msg: message from the contact sensor
        :return:
        """
        contact = False
        if len(msg.states) > 0:
            self.contact = 1
        else:
            self.contact = 0


    def getContact(self):
        contact = self.contact
        return contact

    def jointLimitController(self, angle_array):
        
        reached_limit = False
        if (angle_array[0] < self.base_coxa_joint_limit[0]) or (angle_array[0] > self.base_coxa_joint_limit[1]):
            rospy.loginfo("Base coxa joint limit reached")
            reached_limit = True

        if (angle_array[1] < self.coxa_femur_joint_limit[0]) or (angle_array[1] > self.coxa_femur_joint_limit[1]):
            rospy.loginfo("Coxa femur joint limit reached")
            reached_limit = True

        if (angle_array[2] < self.femur_tibia_joint_limit[0]) or (angle_array[2] > self.femur_tibia_joint_limit[1]):
            rospy.loginfo("Femur tibia joint limit reached")
            reached_limit = True
        return reached_limit
    
    def set_pose(self, pose_array:list=[0,0,0]):
        angle_array = self.ik.IK(pose_array)
        if angle_array is not None:
            reach_warn = self.jointLimitController(angle_array)
        self.setAngles(angle_array)

        return reach_warn

    
    def check_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        # rate = rospy.Rate(10)  # 10hz
        while (self.base_coxa_joint_pub.get_num_connections() ==    0):
            rospy.loginfo(f"No subscribers to leg{str(self.leg)} base_coxa_joint_pub yet so we wait and try again")
        rospy.loginfo(f"{self.leg} base_coxa_joint_pub Publisher Connected")

        while (self.coxa_femur_joint_pub.get_num_connections() == 0):
            rospy.loginfo(f"No subscribers to leg{str(self.leg)} coxa_femur_joint_pub yet so we wait and try again")
        rospy.loginfo(f"{self.leg} coxa_femur_joint_pub Publisher Connected")

        while (self.femur_tibia_joint_pub.get_num_connections() == 0):
            rospy.loginfo(f"No subscribers to leg{str(self.leg)} femur_tibia_joint_pub yet so we wait and try again")
        rospy.loginfo(f"{self.leg} femur_tibia_joint_pub Publisher Connected")

        rospy.loginfo("All Joint Publishers READY")
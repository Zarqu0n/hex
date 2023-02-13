#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64

class JointPub():
    def __init__(self,leg):

        self.publishers_array = []
        self.base_coxa_joint_pub = rospy.Publisher('/hexapod/base_coxa_joint_{}_position_controller/command'.format(leg), Float64, queue_size=1)
        self.coxa_femur_joint_pub = rospy.Publisher('/hexapod/coxa_femur_joint_{}_position_controller/command'.format(leg), Float64, queue_size=1)
        self.femur_tibia_joint_pub = rospy.Publisher('/hexapod/femur_tibia_joint_{}_position_controller/command'.format(leg), Float64, queue_size=1)
        
        self.publishers_array.append(self.base_coxa_joint_pub)
        self.publishers_array.append(self.coxa_femur_joint_pub)
        self.publishers_array.append(self.femur_tibia_joint_pub)

    def __move_joints(self, joints_array):
        i = 0
        for publisher_object in self.publishers_array:
          joint_value = Float64()
          joint_value.data = joints_array[i]
          rospy.loginfo(str(joint_value))
          publisher_object.publish(joint_value)
          i += 1


    def move_angle(self, pos=[0,0,0] , rate_value = 2.0):
        rospy.init_node('joint_publisher_node')
        rospy.loginfo("Start Loop")
        rate = rospy.Rate(rate_value)
        self.move_joints(pos)
        rate.sleep()

#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64

class JointPub():
    def __init__(self,leg):

        self.publishers_array = []
        self._haa_joint_pub = rospy.Publisher('/hexapod/haa_joint_{}_position_controller/command'.format(leg), Float64, queue_size=1)
        self._hfe_joint_pub = rospy.Publisher('/hexapod/hfe_joint_{}_position_controller/command'.format(leg), Float64, queue_size=1)
        self._kfe_joint_pub = rospy.Publisher('/hexapod/kfe_joint_{}_position_controller/command'.format(leg), Float64, queue_size=1)
        
        self.publishers_array.append(self._haa_joint_pub)
        self.publishers_array.append(self._hfe_joint_pub)
        self.publishers_array.append(self._kfe_joint_pub)

    def move_joints(self, joints_array):

        i = 0
        for publisher_object in self.publishers_array:
          joint_value = Float64()
          joint_value.data = joints_array[i]
          rospy.loginfo(str(joint_value))
          publisher_object.publish(joint_value)
          i += 1


    def start_loop(self, rate_value = 2.0, pos1=[0,0,0] ,pos2=[0,0,0]):
        rospy.init_node('joint_publisher_node')
        rospy.loginfo("Start Loop")
        position = "pos1"
        rate = rospy.Rate(rate_value)

        if position == "pos1":
          self.move_joints(pos1)
          position = "pos2"
        else:
          self.move_joints(pos2)
          position = "pos1"
          rate.sleep()

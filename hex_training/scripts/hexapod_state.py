#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3
from sensor_msgs.msg import JointState
import tf
import numpy
import math
from hex_control.legCtrl import LegController
from std_msgs.msg import Int32
    
class HexapodState(object):

    def __init__(self,abs_max_roll, abs_max_pitch,abs_min_pos_dist = 0.1, abs_max_pos_dist = 0.5 ,
                 done_reward = -1000.0, alive_reward=10.0, desired_yaw=0.0, abs_max_distance=10.0,
                 weight_r1=5.0, weight_r2=-0.5, weight_r3=1.0,
                 coxa_joint_max=2*numpy.pi, coxa_joint_min=0.0, femur_joint_max=2*numpy.pi, femur_joint_min=0.0,
                   tibia_joint_max=2*numpy.pi, tibia_joint_min=0.0,debug=True):
        

        #Topics
        self.odom_topic = "/odom"
        self.imu_topic = "/imu"
        self.joint_state_topic = "/hexapod/joint_states"


        rospy.loginfo("Subscribing to odom and imu topics...")
        rospy.Subscriber(self.odom_topic, Odometry, self.odomCb)
        # We use the IMU for orientation and linearacceleration detection
        rospy.Subscriber(self.imu_topic, Imu, self.imuCb)
        # We use it to get the contact force, to know if its in the air or stumping too hard.
        
        rospy.loginfo("Subscribing to joint_states topic...")
        # We use it to get the joints positions and calculate the reward associated to it
        rospy.Subscriber(self.joint_state_topic, JointState, self.joints_state_callback)
        self.joints_state = rospy.wait_for_message(self.joint_state_topic, JointState)

        rospy.loginfo("Starting MonopedState Class object...")
        self.desired_world_point = Vector3(0.0, 0.0, 0.0)

        self._abs_max_roll = abs_max_roll
        self._abs_max_pitch = abs_max_pitch
        self._abs_min_pos_dist = abs_min_pos_dist
        self._abs_max_pos_dist = abs_max_pos_dist
        self._done_reward = done_reward
        self._alive_reward = alive_reward
        self._desired_yaw = desired_yaw
        self.abs_max_distance = abs_max_distance
        self.coxa_joint_max = coxa_joint_max
        self.coxa_joint_min = coxa_joint_min
        self.femur_joint_max = femur_joint_max
        self.femur_joint_min = femur_joint_min
        self.tibia_joint_max = tibia_joint_max
        self.tibia_joint_min = tibia_joint_min
        self._weight_r1 = weight_r1
        self._weight_r2 = weight_r2
        self._weight_r3 = weight_r3
        self._debug = debug

        self._list_of_observations = ["distance_x_from_desired_point",
                "distance_y_from_desired_point",
                "distance_z_from_desired_point",
                "base_roll",
                "base_pitch",
                "base_yaw",
                "contact_bool_1",
                "contact_bool_2",
                "contact_bool_3",
                "contact_bool_4",
                "contact_bool_5",
                "contact_bool_6",
                "base_coxa_joint_1_state",
                "coxa_femur_joint_1_state",
                "femur_tibia_joint_1_state",
                "base_coxa_joint_2_state",
                "coxa_femur_joint_2_state",
                "femur_tibia_joint_2_state",
                "base_coxa_joint_3_state",
                "coxa_femur_joint_3_state",
                "femur_tibia_joint_3_state",
                "base_coxa_joint_4_state",
                "coxa_femur_joint_4_state",
                "femur_tibia_joint_4_state",
                "base_coxa_joint_5_state",
                "coxa_femur_joint_5_state",
                "femur_tibia_joint_5_state",
                "base_coxa_joint_6_state",
                "coxa_femur_joint_6_state",
                "femur_tibia_joint_6_state"]

        # We init the observation ranges and We create the bins now for all the observations

        self.base_position = Point()
        self.base_orientation = Quaternion()
        self.base_linear_acceleration = Vector3()
        self.legs = [LegController(i) for i in range(1,7)]
        self.contacts = list()

        self.action_positions = numpy.zeros(18)

        self.distance_reward = 0.0
        self.orientation_reward = 0.0
        self.contact_reward = 0.0
        self.total_reward = 0.0

        self.step = 0
        rospy.loginfo("MonopedState Class object initialization done.")

    def check_all_systems_ready(self):
        """
        We check that all systems are ready
        :return:
        """
        data_pose = None
        while data_pose is None and not rospy.is_shutdown():
            try:
                data_pose = rospy.wait_for_message(self.odom_topic, Odometry, timeout=0.1)
                self.base_position = data_pose.pose.pose.position
                rospy.loginfo("Current odom READY")
            except:
                rospy.loginfo("Current odom pose not ready yet, retrying for getting robot base_position")

        imu_data = None
        while imu_data is None and not rospy.is_shutdown():
            try:
                imu_data = rospy.wait_for_message(self.imu_topic, Imu, timeout=0.1)
                self.base_orientation = imu_data.orientation
                self.base_linear_acceleration = imu_data.linear_acceleration
                rospy.loginfo("Current imu_data READY")
            except:
                rospy.loginfo("Current imu_data not ready yet, retrying for getting robot base_orientation, and base_linear_acceleration")

        contacts_data = list()
        while len(contacts_data) != 0 and not rospy.is_shutdown():
            try:
                for i in range(1,7):
                    contacts_data.append(self.legs[i].getContact())
                rospy.loginfo("Current contacts_data READY")
            except:
                rospy.loginfo("Current contacts_data not ready yet, retrying")

        joint_states_msg = None
        while joint_states_msg is None and not rospy.is_shutdown():
            try:
                joint_states_msg = rospy.wait_for_message(self.joint_state_topic, JointState, timeout=0.1)
                self.joints_state = joint_states_msg
                rospy.loginfo("Current joint_states READY")
            except Exception as e:
                rospy.loginfo("Current joint_states not ready yet, retrying==>"+str(e))

        rospy.loginfo("ALL SYSTEMS READY")

    def setDesiredWorldPoints(self, x, y, z):
        """
        Point where you want the Monoped to be
        :return:
        """
        self.desired_world_point.x = x
        self.desired_world_point.y = y
        self.desired_world_point.z = z

    def getBaseRPY(self):
        euler_rpy = Vector3()
        euler = tf.transformations.euler_from_quaternion(
            [self.base_orientation.x, self.base_orientation.y, self.base_orientation.z, self.base_orientation.w])

        euler_rpy.x = euler[0]
        euler_rpy.y = euler[1]
        euler_rpy.z = euler[2]
        return euler_rpy

    def getDistanceFromPoints(self, p_end):
        """
        Given a Vector3 Object, get distance from current position
        :param p_end:
        :return:
        """
        a = numpy.array((self.base_position.x, self.base_position.y, self.base_position.z))
        b = numpy.array((p_end.x, p_end.y, p_end.z))

        distance = numpy.linalg.norm(a - b)

        return distance

    def getDifferenceDistanceFromPoints(self, p_end):
        """
        Given a Vector3 Object, get distance from current position
        :param p_end:
        :return:
        """
        a = numpy.array((self.base_position.x, self.base_position.y, self.base_position.z))
        b = numpy.array((p_end.x, p_end.y, p_end.z))

        distance = a - b

        return distance
    
    def getLegsContact(self):
        """
        Get the contact state of all legs
        :return contact: list of bool
        """
        for leg in self.legs:
            self.contacts.append(leg.getContact())
    
        return self.contacts

    def getJointStates(self):
        return self.joints_state

    def odomCb(self,msg):
        self.base_position = msg.pose.pose.position

    def imuCb(self,msg):
        self.base_orientation = msg.orientation
        self.base_linear_acceleration = msg.linear_acceleration


    def joints_state_callback(self,msg):
        self.joints_state = msg

    # def monoped_height_ok(self):

    #     height_ok = self._min_height <= self.get_base_height() < self._max_height
    #     return height_ok

    def isPositionOk(self):
        distance = self.getDistanceFromPoints(self.desired_world_point)
        position_ok = self._abs_min_pos_dist <= distance < self._abs_max_pos_dist
        return position_ok

    def isBaseOriantationOk(self):

        orientation_rpy = self.getBaseRPY()
        roll_ok = self._abs_max_roll > abs(orientation_rpy.x)
        pitch_ok = self._abs_max_pitch > abs(orientation_rpy.y)
        orientation_ok = roll_ok and pitch_ok
        return orientation_ok


    def rewardContact(self, weight=1.0):
        """
        We calculate reward base on the contact.
        Tibia link contact with link is better at the end of action
        :return:
        """
        reward = 0.0
        # Abs to remove sign
        for leg in self.legs:
            contact = leg.getContact()
            reward += weight * contact
        return reward

    def rewardOriantation(self, weight=1.0):
        """
        We calculate the reward based on the orientation.
        The more its closser to 0 the better because it means its upright
        desired_yaw is the yaw that we want it to be.
        to praise it to have a certain orientation, here is where to set it.
        :return:
        """
        curren_orientation = self.getBaseRPY()
        yaw_displacement = curren_orientation.z - self._desired_yaw
        acumulated_orientation_displacement = abs(curren_orientation.x) + abs(curren_orientation.y) + abs(yaw_displacement)
        reward = -weight * acumulated_orientation_displacement
        return reward

    def rewardDistance(self, weight=1.0):
        """
        We calculate the distance from the desired point.
        The closser the better
        :param weight:
        :return:reward
        """
        distance = self.getDistanceFromPoints(self.desired_world_point)
        reward = -weight * distance
        return reward

    def totalReward(self):
        """
        """

        self.distance_reward = self.rewardDistance(self._weight_r1)
        self.contact_reward = self.rewardContact(self._weight_r2)
        self.orientation_reward = self.rewardOriantation(self._weight_r3)

        # The sign depend on its function.
        self.total_reward = self._alive_reward + self.distance_reward + self.contact_reward + self.orientation_reward

        return self.total_reward
        

    def action2pose(self, action):
        pose_matris = numpy.zeros((6, 3))
        for i in range(len(self.legs)):
            pose_matris[i] = [action[i], action[i+6], action[i+12]]
        return pose_matris
    
    def getObservations(self):
        """
        Returns the state of the robot needed for OpenAI QLearn Algorithm
        The state will be defined by an array of the:
        1) distance from desired x point in meters
        2) distance from desired y point in meters
        3) distance from desired z point in meters
        4) The roll orientation in radians
        5) the pitch orientation in radians
        6) the Yaw orientation in radians
        7) Force in leg 1's contact sensor in bool
        8) Force in leg 2's contact sensor in bool
        9) Force in leg 3's contact sensor in bool
        10) Force in leg 4's contact sensor in bool
        11) Force in leg 5's contact sensor in bool
        12) Force in leg 6's contact sensor in bool
        13) Base coxa joint 1 state in radians
        14) Coxa femur joint 1 state in radians
        15) Femur tibia joint 1 state in radians
        16) Base coxa joint 2 state in radians
        17) Coxa femur joint 2 state in radians
        18) Femur tibia joint 2 state in radians
        19) Base coxa joint 3 state in radians
        20) Coxa femur joint 3 state in radians
        21) Femur tibia joint 3 state in radians
        22) Base coxa joint 4 state in radians
        23) Coxa femur joint 4 state in radians
        24) Femur tibia joint 4 state in radians
        25) Base coxa joint 5 state in radians
        26) Coxa femur joint 5 state in radians
        27) Femur tibia joint 5 state in radians
        28) Base coxa joint 6 state in radians
        29) Coxa femur joint 6 state in radians
        30) Femur tibia joint 6 state in radians


        observation = ["distance_x_from_desired_point",
                "distance_y_from_desired_point",
                "distance_z_from_desired_point",
                "base_roll",
                "base_pitch",
                "base_yaw",
                "contact_bool_1",
                "contact_bool_2",
                "contact_bool_3",
                "contact_bool_4",
                "contact_bool_5",
                "contact_bool_6",
                "base_coxa_joint_1_state",
                "coxa_femur_joint_1_state",
                "femur_tibia_joint_1_state",
                "base_coxa_joint_2_state",
                "coxa_femur_joint_2_state",
                "femur_tibia_joint_2_state",
                "base_coxa_joint_3_state",
                "coxa_femur_joint_3_state",
                "femur_tibia_joint_3_state",
                "base_coxa_joint_4_state",
                "coxa_femur_joint_4_state",
                "femur_tibia_joint_4_state",
                "base_coxa_joint_5_state",
                "coxa_femur_joint_5_state",
                "femur_tibia_joint_5_state",
                "base_coxa_joint_6_state",
                "coxa_femur_joint_6_state",
                "femur_tibia_joint_6_state"]

        :return: observation
        """

        [distance_x_from_desired_point,
         distance_y_from_desired_point,distance_z_from_desired_point] = self.getDifferenceDistanceFromPoints(self.desired_world_point)

        base_orientation = self.getBaseRPY()
        base_roll = base_orientation.x
        base_pitch = base_orientation.y
        base_yaw = base_orientation.z

        contacts = self.getLegsContact()
        contact_bool_1 = contacts[0]
        contact_bool_2 = contacts[1]
        contact_bool_3 = contacts[2]
        contact_bool_4 = contacts[3]
        contact_bool_5 = contacts[4]
        contact_bool_6 = contacts[5]

        joint_states = self.getJointStates()
        base_coxa_joint_1_state = joint_states.position[0]
        base_coxa_joint_2_state = joint_states.position[1]
        base_coxa_joint_3_state = joint_states.position[2]
        base_coxa_joint_4_state = joint_states.position[3]
        base_coxa_joint_5_state = joint_states.position[4]
        base_coxa_joint_6_state = joint_states.position[5]
        coxa_femur_joint_1_state = joint_states.position[6]
        coxa_femur_joint_2_state = joint_states.position[7]
        coxa_femur_joint_3_state = joint_states.position[8]
        coxa_femur_joint_4_state = joint_states.position[9]
        coxa_femur_joint_5_state = joint_states.position[10]
        coxa_femur_joint_6_state = joint_states.position[11]
        femur_tibia_joint_1_state = joint_states.position[12]
        femur_tibia_joint_2_state = joint_states.position[13]
        femur_tibia_joint_3_state = joint_states.position[14]
        femur_tibia_joint_4_state = joint_states.position[15]
        femur_tibia_joint_5_state = joint_states.position[16]
        femur_tibia_joint_6_state = joint_states.position[17]



        observation = {"Distance":numpy.array([distance_x_from_desired_point,
         distance_y_from_desired_point,distance_z_from_desired_point]),
             "BaseOrientation":numpy.array([base_roll,base_pitch,base_yaw]),
             "Contacts":numpy.array([contact_bool_1,contact_bool_2,contact_bool_3,contact_bool_4,contact_bool_5,contact_bool_6]),
             "JointStates":numpy.array([base_coxa_joint_1_state,base_coxa_joint_2_state,base_coxa_joint_3_state,base_coxa_joint_4_state,base_coxa_joint_5_state,base_coxa_joint_6_state,
             coxa_femur_joint_1_state,coxa_femur_joint_2_state,coxa_femur_joint_3_state,coxa_femur_joint_4_state,coxa_femur_joint_5_state,coxa_femur_joint_6_state,
             femur_tibia_joint_1_state,femur_tibia_joint_2_state,femur_tibia_joint_3_state,femur_tibia_joint_4_state,femur_tibia_joint_5_state,femur_tibia_joint_6_state])}

        return observation

    def processData(self):
        """
        We return the total reward based on the state in which we are in and if its done or not
        ( it fell basically )
        :return: reward, done
        """
        position_ok = self.isPositionOk()
        orientation_ok = self.isBaseOriantationOk()
        self.step += 1
        done = (position_ok and orientation_ok)
        if done:
            rospy.loginfo("It fell, so the reward has to be very low")
            total_reward = self._done_reward
        else:
            rospy.loginfo("Calculate normal reward because it didn't fall.")
            total_reward = self.totalReward()

        distance = self.getDistanceFromPoints(self.desired_world_point)
        info = {"Total Reward":total_reward,
                "Distance":distance}

        if self._debug:
            for i in range(100):
                rospy.loginfo("")
            
            epoch = rospy.wait_for_message("/train_stats/epoch", Int32)
            rospy.loginfo("Epoch: %d", epoch.data)
            rospy.loginfo("Step: %d", self.step)
            rospy.loginfo("-Total reward: %f", total_reward)
            rospy.loginfo("--Distance reward: %f", self.distance_reward)
            rospy.loginfo("--Orientation reward: %f", self.orientation_reward)
            rospy.loginfo("--Contact reward: %f", self.contact_reward)
            rospy.loginfo("-Distance: %f", distance)
            rospy.loginfo("-Done: %s", done)

        return total_reward, done, info

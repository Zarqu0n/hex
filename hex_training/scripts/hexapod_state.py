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

    
class HexapodState(object):

    def __init__(self,abs_max_roll, abs_max_pitch,abs_min_pos_dist = 0.1, abs_max_pos_dist = 0.5 ,joint_increment_value = 0.05, 
                 done_reward = -1000.0, alive_reward=10.0, desired_yaw=0.0, abs_max_distance=10.0,
                 weight_r1=5.0, weight_r2=-0.5, weight_r3=1.0,  discrete_division=10,
                 coxa_joint_max=2*numpy.pi, coxa_joint_min=0.0, femur_joint_max=2*numpy.pi, femur_joint_min=0.0,
                   tibia_joint_max=2*numpy.pi, tibia_joint_min=0.0):
        

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
        self._joint_increment_value = joint_increment_value
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

        #TODO Added other joints
        self._list_of_observations = ["distance_x_from_desired_point",
                "distance_y_from_desired_point",
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


        self._discrete_division = discrete_division
        # We init the observation ranges and We create the bins now for all the observations
        self.initBins()

        self.base_position = Point()
        self.base_orientation = Quaternion()
        self.base_linear_acceleration = Vector3()
        self.legs = [LegController(i) for i in range(1,7)]
        self.contacts = list()
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

    # def contact_callback(self,msg):
    #     """
    #     /lowerleg_contactsensor_state/states[0]/contact_positions ==> PointContact in World
    #     /lowerleg_contactsensor_state/states[0]/contact_normals ==> NormalContact in World

    #     ==> One is an array of all the forces, the other total,
    #      and are relative to the contact link referred to in the sensor.
    #     /lowerleg_contactsensor_state/states[0]/wrenches[]
    #     /lowerleg_contactsensor_state/states[0]/total_wrench
    #     :param msg:
    #     :return:
    #     """
    #     for state in msg.states:
    #         self.contact_force = state.total_wrench.force

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

    # def calculate_reward_joint_position(self, weight=1.0):
    #     """
    #     We calculate reward base on the joints configuration. The more near 0 the better.
    #     :return:
    #     """
    #     acumulated_joint_pos = 0.0
    #     for joint_pos in self.joints_state.position:
    #         # Abs to remove sign influence, it doesnt matter the direction of turn.
    #         acumulated_joint_pos += abs(joint_pos)
    #         rospy.loginfo("calculate_reward_joint_position>>acumulated_joint_pos=" + str(acumulated_joint_pos))
    #     reward = weight * acumulated_joint_pos
    #     rospy.loginfo("calculate_reward_joint_position>>reward=" + str(reward))
    #     return reward

    # def calculate_reward_joint_effort(self, weight=1.0):
    #     """
    #     We calculate reward base on the joints effort readings. The more near 0 the better.
    #     :return:
    #     """
    #     acumulated_joint_effort = 0.0
    #     for joint_effort in self.joints_state.effort:
    #         # Abs to remove sign influence, it doesnt matter the direction of the effort.
    #         acumulated_joint_effort += abs(joint_effort)
    #         rospy.loginfo("calculate_reward_joint_effort>>joint_effort=" + str(joint_effort))
    #         rospy.loginfo("calculate_reward_joint_effort>>acumulated_joint_effort=" + str(acumulated_joint_effort))
    #     reward = weight * acumulated_joint_effort
    #     rospy.loginfo("calculate_reward_joint_effort>>reward=" + str(reward))
    #     return reward

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
        rospy.loginfo("calculate_reward_contact_force>>reward=" + str(reward))
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
        rospy.loginfo("calculate_reward_orientation>>[R,P,Y]=" + str(curren_orientation))
        acumulated_orientation_displacement = abs(curren_orientation.x) + abs(curren_orientation.y) + abs(yaw_displacement)
        reward = weight * acumulated_orientation_displacement
        rospy.loginfo("calculate_reward_orientation>>reward=" + str(reward))
        return reward

    def rewardDistance(self, weight=1.0):
        """
        We calculate the distance from the desired point.
        The closser the better
        :param weight:
        :return:reward
        """
        distance = self.getDistanceFromPoints(self.desired_world_point)
        reward = weight * distance
        rospy.loginfo("calculate_reward_orientation>>reward=" + str(reward))
        return reward

    def totalReward(self):
        """
        We consider VERY BAD REWARD -7 or less
        Perfect reward is 0.0, and total reward 1.0.
        The defaults values are chosen so that when the robot has fallen or very extreme joint config:
        r1 = -8.04
        r2 = -8.84
        r3 = -7.08
        r4 = -10.0 ==> We give priority to this, giving it higher value.
        :return:
        """

        alive_reward = self._alive_reward
        r1 = self.rewardDistance(self._weight_r1)
        r2 = self.rewardContact(self._weight_r2)
        r3 = self.rewardOriantation(self._weight_r3)

        # The sign depend on its function.
        total_reward = alive_reward - r1 - r2 - r3

        rospy.loginfo("###############")
        rospy.loginfo("alive_bonus=" + str(self._alive_reward))
        rospy.loginfo("r1 rewardDistance=" + str(r1))
        rospy.loginfo("r2 rewardContact=" + str(r2))
        rospy.loginfo("r3 rewardOriantation=" + str(r3))
        rospy.loginfo("total_reward=" + str(total_reward))
        rospy.loginfo("###############")

        return total_reward
        

    def getObservations(self):
        """
        Returns the state of the robot needed for OpenAI QLearn Algorithm
        The state will be defined by an array of the:
        1) distance from desired x point in meters
        2) distance from desired y point in meters
        3) The roll orientation in radians
        4) the pitch orientation in radians
        5) the Yaw orientation in radians
        6) Force in leg 1's contact sensor in bool
        7) Force in leg 2's contact sensor in bool
        8) Force in leg 3's contact sensor in bool
        9) Force in leg 4's contact sensor in bool
        10) Force in leg 5's contact sensor in bool
        11) Force in leg 6's contact sensor in bool
        12) Base coxa joint 1 state in radians
        13) Coxa femur joint 1 state in radians
        14) Femur tibia joint 1 state in radians
        15) Base coxa joint 2 state in radians
        16) Coxa femur joint 2 state in radians
        17) Femur tibia joint 2 state in radians
        18) Base coxa joint 3 state in radians
        19) Coxa femur joint 3 state in radians
        20) Femur tibia joint 3 state in radians
        21) Base coxa joint 4 state in radians
        22) Coxa femur joint 4 state in radians
        23) Femur tibia joint 4 state in radians
        24) Base coxa joint 5 state in radians
        25) Coxa femur joint 5 state in radians
        26) Femur tibia joint 5 state in radians
        27) Base coxa joint 6 state in radians
        28) Coxa femur joint 6 state in radians
        29) Femur tibia joint 6 state in radians


        observation = ["distance_x_from_desired_point",
                "distance_y_from_desired_point",
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
         distance_y_from_desired_point,_] = self.getDifferenceDistanceFromPoints(self.desired_world_point)

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


        observation = []
        for obs_name in self._list_of_observations:
            if obs_name == "distance_x_from_desired_point":
                observation.append(distance_x_from_desired_point)
            elif obs_name == "distance_y_from_desired_point":
                observation.append(distance_y_from_desired_point)
            elif obs_name == "base_roll":
                observation.append(base_roll)
            elif obs_name == "base_pitch":
                observation.append(base_pitch)
            elif obs_name == "base_yaw":
                observation.append(base_yaw)
            elif obs_name == "contact_bool_1":
                observation.append(contact_bool_1)
            elif obs_name == "contact_bool_2":
                observation.append(contact_bool_2)
            elif obs_name == "contact_bool_3":
                observation.append(contact_bool_3)
            elif obs_name == "contact_bool_4":
                observation.append(contact_bool_4)
            elif obs_name == "contact_bool_5":
                observation.append(contact_bool_5)
            elif obs_name == "contact_bool_6":
                observation.append(contact_bool_6)
            elif obs_name == "base_coxa_joint_1_state":
                observation.append(base_coxa_joint_1_state)
            elif obs_name == "coxa_femur_joint_1_state":
                observation.append(coxa_femur_joint_1_state)
            elif obs_name == "femur_tibia_joint_1_state":
                observation.append(femur_tibia_joint_1_state)
            elif obs_name == "base_coxa_joint_2_state":
                observation.append(base_coxa_joint_2_state)
            elif obs_name == "coxa_femur_joint_2_state":
                observation.append(coxa_femur_joint_2_state)
            elif obs_name == "femur_tibia_joint_2_state":
                observation.append(femur_tibia_joint_2_state)
            elif obs_name == "base_coxa_joint_3_state":
                observation.append(base_coxa_joint_3_state)
            elif obs_name == "coxa_femur_joint_3_state":
                observation.append(coxa_femur_joint_3_state)
            elif obs_name == "femur_tibia_joint_3_state":
                observation.append(femur_tibia_joint_3_state)
            elif obs_name == "base_coxa_joint_4_state":
                observation.append(base_coxa_joint_4_state)
            elif obs_name == "coxa_femur_joint_4_state":
                observation.append(coxa_femur_joint_4_state)
            elif obs_name == "femur_tibia_joint_4_state":
                observation.append(femur_tibia_joint_4_state)
            elif obs_name == "base_coxa_joint_5_state":
                observation.append(base_coxa_joint_5_state)
            elif obs_name == "coxa_femur_joint_5_state":
                observation.append(coxa_femur_joint_5_state)
            elif obs_name == "femur_tibia_joint_5_state":
                observation.append(femur_tibia_joint_5_state)
            elif obs_name == "base_coxa_joint_6_state":
                observation.append(base_coxa_joint_6_state)
            elif obs_name == "coxa_femur_joint_6_state":
                observation.append(coxa_femur_joint_6_state)
            elif obs_name == "femur_tibia_joint_6_state":
                observation.append(femur_tibia_joint_6_state)
            else:
                raise NameError('Observation Asked does not exist=='+str(obs_name))

        return observation

    def getStateAsString(self, observation):
        """
        This function will do two things:
        1) It will make discrete the observations
        2) Will convert the discrete observations in to state tags strings
        :param observation:
        :return: state
        """
        observations_discrete = self.assignBins(observation)
        string_state = ''.join(map(str, observations_discrete))
        return string_state

    def assignBins(self, observation):
        """
        Will make observations discrete by placing each value into its corresponding bin
        :param observation:
        :return:
        """
        state_discrete = numpy.zeros(len(self._list_of_observations))
        for i in range(len(self._list_of_observations)):
            state_discrete[i] = numpy.digitize(observation[i], self._bins[i])
        return state_discrete

    def initBins(self):
        """
        We initalise all related to the bins
        :return:
        """
        self.fillObservationsRanges()
        self.createBins()

    def fillObservationsRanges(self):
        """
        We create the dictionary for the ranges of the data related to each observation
        :return:
        """
        #TODO add other joints
        self._obs_range_dict = {}
        for obs_name in self._list_of_observations:
            if obs_name == "distance_x_from_desired_point":
                max_value = self.abs_max_distance
                min_value = -self.abs_max_distance
            elif obs_name == "distance_y_from_desired_point":
                max_value = self.abs_max_distance
                min_value = -self.abs_max_distance
            elif obs_name == "base_roll":
                max_value = self._abs_max_roll
                min_value = -self._abs_max_roll
            elif obs_name == "base_pitch":
                max_value = self._abs_max_pitch
                min_value = -self._abs_max_pitch
            elif obs_name == "base_yaw":
                max_value = 2*math.pi
                min_value = -2*math.pi
            elif obs_name == "contact_bool_1":
                max_value = 1
                min_value = 0
            elif obs_name == "contact_bool_2":
                max_value = 1
                min_value = 0
            elif obs_name == "contact_bool_3":
                max_value = 1
                min_value = 0
            elif obs_name == "contact_bool_4":
                max_value = 1
                min_value = 0
            elif obs_name == "contact_bool_5":
                max_value = 1
                min_value = 0
            elif obs_name == "contact_bool_6":
                max_value = 1
                min_value = 0
            elif obs_name == "base_coxa_joint_1_state":
                max_value = self.coxa_joint_max
                min_value = self.coxa_joint_min
            elif obs_name == "coxa_femur_joint_1_state":
                max_value = self.femur_joint_max
                min_value = self.femur_joint_min
            elif obs_name == "femur_tibia_joint_1_state":
                max_value = self.tibia_joint_max
                min_value = self.tibia_joint_min
            elif obs_name == "base_coxa_joint_2_state":
                max_value = self.coxa_joint_max
                min_value = self.coxa_joint_min
            elif obs_name == "coxa_femur_joint_2_state":
                max_value = self.femur_joint_max
                min_value = self.femur_joint_min
            elif obs_name == "femur_tibia_joint_2_state":
                max_value = self.tibia_joint_max
                min_value = self.tibia_joint_min
            elif obs_name == "base_coxa_joint_3_state":
                max_value = self.coxa_joint_max
                min_value = self.coxa_joint_min
            elif obs_name == "coxa_femur_joint_3_state":
                max_value = self.femur_joint_max
                min_value = self.femur_joint_min
            elif obs_name == "femur_tibia_joint_3_state":
                max_value = self.tibia_joint_max
                min_value = self.tibia_joint_min
            elif obs_name == "base_coxa_joint_4_state":
                max_value = self.coxa_joint_max
                min_value = self.coxa_joint_min
            elif obs_name == "coxa_femur_joint_4_state":
                max_value = self.femur_joint_max
                min_value = self.femur_joint_min
            elif obs_name == "femur_tibia_joint_4_state":
                max_value = self.tibia_joint_max
                min_value = self.tibia_joint_min
            elif obs_name == "base_coxa_joint_5_state":
                max_value = self.coxa_joint_max
                min_value = self.coxa_joint_min
            elif obs_name == "coxa_femur_joint_5_state":
                max_value = self.femur_joint_max
                min_value = self.femur_joint_min
            elif obs_name == "femur_tibia_joint_5_state":
                max_value = self.tibia_joint_max
                min_value = self.tibia_joint_min
            elif obs_name == "base_coxa_joint_6_state":
                max_value = self.coxa_joint_max
                min_value = self.coxa_joint_min
            elif obs_name == "coxa_femur_joint_6_state":
                max_value = self.femur_joint_max
                min_value = self.femur_joint_min
            elif obs_name == "femur_tibia_joint_6_state":
                max_value = self.tibia_joint_max
                min_value = self.tibia_joint_min
            else:
                raise NameError('Observation Asked does not exist=='+str(obs_name))

            self._obs_range_dict[obs_name] = [min_value,max_value]

    def createBins(self):
        """
        We create the Bins for the discretization of the observations
        self.desired_world_point = Vector3(0.0, 0.0, 0.0)
        self._min_height = min_height
        self._max_height = max_height
        self._abs_max_roll = abs_max_roll
        self._abs_max_pitch = abs_max_pitch
        self._joint_increment_value = joint_increment_value
        self._done_reward = done_reward
        self._alive_reward = alive_reward
        self._desired_force = desired_force
        self._desired_yaw = desired_yaw


        :return:bins
        """

        number_of_observations = len(self._list_of_observations)
        parts_we_disrcetize = self._discrete_division

        self._bins = numpy.zeros((number_of_observations, parts_we_disrcetize))
        for counter in range(number_of_observations):
            obs_name = self._list_of_observations[counter]
            min_value = self._obs_range_dict[obs_name][0]
            max_value = self._obs_range_dict[obs_name][1]
            self._bins[counter] = numpy.linspace(min_value, max_value, parts_we_disrcetize)

    def get_action_to_position(self, action):
        """
        Here we have the ACtions number to real joint movement correspondance.
        :param action: Integer that goes from 0 to 35, because we have 36 actions.
        Each legs 3 increment and 3 decrement.
        :return:
        """
        # We get current Joints values
        joint_states = self.getJointStates()
        joint_states_position = joint_states.position
        
        #action_position = [0.0,0.0,...,0.0]( has 36 0.0)
        action_positions = [0.0 for i in range(len(joint_states_position))]

        rospy.loginfo("get_action_to_position>>>"+str(joint_states_position))

        for i in range(len(joint_states_position)):
            action_positions[i] = joint_states_position[i]

        if action == 0: #Increment base_coxa_joint_1           
            action_positions[0] += self._joint_increment_value
        elif action == 1: #Decrement base_coxa_joint_1
            action_positions[0] -= self._joint_increment_value
        elif action == 2: #Increment base_coxa_joint_2
            action_positions[1] += self._joint_increment_value
        elif action == 3: #Decrement base_coxa_joint_2
            action_positions[1] -= self._joint_increment_value
        elif action == 4: #Increment base_coxa_joint_3
            action_positions[2] += self._joint_increment_value
        elif action == 5: #Decrement base_coxa_joint_3
            action_positions[2] -= self._joint_increment_value
        elif action == 6: #Increment base_coxa_joint_4
            action_positions[3] += self._joint_increment_value
        elif action == 7: #Decrement base_coxa_joint_4
            action_positions[3] -= self._joint_increment_value
        elif action == 8: #Increment base_coxa_joint_5
            action_positions[4] += self._joint_increment_value
        elif action == 9: #Decrement base_coxa_joint_5
            action_positions[4] -= self._joint_increment_value
        elif action == 10: #Increment base_coxa_joint_6
            action_positions[5] += self._joint_increment_value
        elif action == 11: #Decrement base_coxa_joint_6
            action_positions[5] -= self._joint_increment_value
        elif action == 12: #Increment coxa_femur_joint_1
            action_positions[6] += self._joint_increment_value
        elif action == 13: #Decrement coxa_femur_joint_1
            action_positions[6] -= self._joint_increment_value
        elif action == 14: #Increment coxa_femur_joint_2
            action_positions[7] += self._joint_increment_value
        elif action == 15: #Decrement coxa_femur_joint_2
            action_positions[7] -= self._joint_increment_value
        elif action == 16: #Increment coxa_femur_joint_3
            action_positions[8] += self._joint_increment_value
        elif action == 17: #Decrement coxa_femur_joint_3
            action_positions[8] -= self._joint_increment_value
        elif action == 18: #Increment coxa_femur_joint_4
            action_positions[9] += self._joint_increment_value
        elif action == 19: #Decrement coxa_femur_joint_4
            action_positions[9] -= self._joint_increment_value
        elif action == 20: #Increment coxa_femur_joint_5
            action_positions[10] += self._joint_increment_value
        elif action == 21: #Decrement coxa_femur_joint_5
            action_positions[10] -= self._joint_increment_value
        elif action == 22: #Increment coxa_femur_joint_6
            action_positions[11] += self._joint_increment_value
        elif action == 23: #Decrement coxa_femur_joint_6
            action_positions[11] -= self._joint_increment_value
        elif action == 24: #Increment femur_tibia_joint_1
            action_positions[12] += self._joint_increment_value
        elif action == 25: #Decrement femur_tibia_joint_1
            action_positions[12] -= self._joint_increment_value
        elif action == 26: #Increment femur_tibia_joint_2
            action_positions[13] += self._joint_increment_value
        elif action == 27: #Decrement femur_tibia_joint_2
            action_positions[13] -= self._joint_increment_value
        elif action == 28: #Increment femur_tibia_joint_3
            action_positions[14] += self._joint_increment_value
        elif action == 29: #Decrement femur_tibia_joint_3
            action_positions[14] -= self._joint_increment_value
        elif action == 30: #Increment femur_tibia_joint_4
            action_positions[15] += self._joint_increment_value
        elif action == 31: #Decrement femur_tibia_joint_4
            action_positions[15] -= self._joint_increment_value
        elif action == 32: #Increment femur_tibia_joint_5
            action_positions[16] += self._joint_increment_value
        elif action == 33: #Decrement femur_tibia_joint_5
            action_positions[16] -= self._joint_increment_value
        elif action == 34: #Increment femur_tibia_joint_6
            action_positions[17] += self._joint_increment_value
        elif action == 35: #Decrement femur_tibia_joint_6
            action_positions[17] -= self._joint_increment_value
        return action_positions

    def processData(self):
        """
        We return the total reward based on the state in which we are in and if its done or not
        ( it fell basically )
        :return: reward, done
        """
        position_ok = self.isPositionOk()
        orientation_ok = self.isBaseOriantationOk()

        done = (position_ok and orientation_ok)
        if done:
            rospy.loginfo("It fell, so the reward has to be very low")
            total_reward = self._done_reward
        else:
            rospy.loginfo("Calculate normal reward because it didn't fall.")
            total_reward = self.totalReward()

        return total_reward, done

    def testing_loop(self):

        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.totalReward()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('monoped_state_node', anonymous=True)
    hex_state = HexapodState(abs_max_roll=0.7,
                                 abs_max_pitch=0.7)
    hex_state.testing_loop()

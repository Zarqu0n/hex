#!/usr/bin/env python
'''
    By Miguel Angel Rodriguez <duckfrost@theconstructsim.com>
    Visit our website at www.theconstructsim.com
'''
# import gym
import gymnasium as gym
import rospy
import numpy as np
import time
from gymnasium import spaces
from geometry_msgs.msg import Pose
from gymnasium.utils import seeding
from gazebo_connection import GazeboConnection
from hex_control.legCtrl import LegController
from hexapod_idle_state import HexapodStateIdle
from controllers_connection import ControllersConnection
from hex_control.hegzi import HexapodController
import random



class HexapodIdle(gym.Env):
    metadata = {'render_modes': ['human']}
    def __init__(self):
        
        # We assume that a ROS node has already been created
        # before initialising the environment

        # gets training parameters from param server
        self.desired_pose = Pose()
        self.desired_pose.position.x = rospy.get_param("/desired_pose/x")
        self.desired_pose.position.y = rospy.get_param("/desired_pose/y")
        self.desired_pose.position.z = rospy.get_param("/desired_pose/z")
        self.running_step = rospy.get_param("/running_step")
        self.max_oriantation = rospy.get_param("/max_oriantation")
        self.done_reward = rospy.get_param("/done_reward")
        self.alive_reward = rospy.get_param("/alive_reward")
        self.desired_yaw = rospy.get_param("/desired_yaw")

        self.abs_min_pos_dist = rospy.get_param("/abs_min_pos_dist")
        self.abs_max_pos_dist = rospy.get_param("/abs_max_pos_dist")
        self.abs_max_distance = rospy.get_param("/abs_max_distance")
        self.weight_r1 = rospy.get_param("/weight_r1")
        self.weight_r2 = rospy.get_param("/weight_r2")
        self.weight_r3 = rospy.get_param("/weight_r3")

        self.coxa_joint_max = rospy.get_param("/coxa_joint_max")
        self.coxa_joint_min = rospy.get_param("/coxa_joint_min")
        self.femur_joint_max = rospy.get_param("/femur_joint_max")
        self.femur_joint_min = rospy.get_param("/femur_joint_min")
        self.tibia_joint_max = rospy.get_param("/tibia_joint_max")
        self.tibia_joint_min = rospy.get_param("/tibia_joint_min")

        self.abs_joint_max_action = rospy.get_param("/abs_joint_max_action")
        
        # stablishes connection with simulator


        self.controllers_object = ControllersConnection(namespace="hexapod")

        self.hexapod_state_object = HexapodStateIdle(   abs_max_roll=self.max_oriantation,
                                                    abs_max_pitch=self.max_oriantation,
                                                    abs_min_pos_dist=self.abs_min_pos_dist,
                                                    abs_max_pos_dist=self.abs_max_pos_dist,
                                                    abs_max_distance=self.abs_max_distance,
                                                    done_reward=self.done_reward,
                                                    alive_reward=self.alive_reward,
                                                    desired_yaw=self.desired_yaw,
                                                    weight_r1=self.weight_r1,
                                                    weight_r2=self.weight_r2,
                                                    weight_r3=self.weight_r3,
                                                    coxa_joint_max=self.coxa_joint_max,
                                                    coxa_joint_min=self.coxa_joint_min,
                                                    femur_joint_max=self.femur_joint_max,
                                                    femur_joint_min=self.femur_joint_min,
                                                    tibia_joint_max=self.tibia_joint_max,
                                                    tibia_joint_min=self.tibia_joint_min,
                                                    debug=False
                                                )

        self.hexapod_state_object.setDesiredWorldPoints(self.desired_pose.position.x,
                                                          self.desired_pose.position.y,
                                                          self.desired_pose.position.z)
        
        observations = self.hexapod_state_object.getObservations()
        # self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(len(self.hexapod_state_object.getObservations()),))

        self.observation_space = spaces.Dict(
            {"Distance":spaces.Box(low=-np.inf, high=np.inf, shape=(2,), dtype=np.float32),
             "BaseOrientation":spaces.Box(low=-2*np.pi, high=2*np.pi, shape=(3,), dtype=np.float32),
             "Contacts":spaces.Box(low=-0, high=1, shape=(6,), dtype=np.float32),
             "JointStates":spaces.Box(low=-2*np.pi, high=2*np.pi, shape=(18,), dtype=np.float32)})
        
        self.hex_controller = HexapodController()

        self.action_space = spaces.Box(low=-self.abs_joint_max_action, high=self.abs_joint_max_action, shape=(18,), dtype=np.float32)

        self.reward = 0.0
        self.cumulated_reward = 0.0
        self.old_reward = 0.0
        self.reward_range = (-np.inf, np.inf)

        self.gazebo = GazeboConnection()
        self._seed()

    # A function to initialize the random generator
    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
        
    # Resets the state of the environment and returns an initial observation.
    def reset(self,seed=None,options=None):

        super().reset(seed=seed)

        self.cumulated_reward = 0
        self.hexapod_state_object.step = 0
        # 0st: We pause the Simulator
        rospy.loginfo("Pausing SIM...")

        #create rando positin range low = 0.1 high = 
        self.desired_pose.position.z = random.uniform(0.45, self.abs_max_pos_dist)
        self.hexapod_state_object.setDesiredWorldPoints(self.desired_pose.position.x,
                                                        self.desired_pose.position.y,
                                                        self.desired_pose.position.z)
        rospy.loginfo("Desired Pose: " + str(self.desired_pose.position.z))

        # self.gazebo.pauseSim()

        # 1st: resets the simulation to initial values
        rospy.loginfo("Reset SIM...")
        self.gazebo.resetSim()

        # 2nd: We Set the gravity to 0.0 so that we dont fall when reseting joints
        # It also UNPAUSES the simulation
        rospy.loginfo("Remove Gravity...")
        self.gazebo.change_gravity(0.0, 0.0, 0.0)

        # EXTRA: Reset JoinStateControlers because sim reset doesnt reset TFs, generating time problems
        rospy.loginfo("reset_monoped_joint_controllers...")
        self.controllers_object.reset_hexapod_joint_controllers()

        # 3rd: resets the robot to initial conditions
        rospy.loginfo("set_init_pose...")
        self.hex_controller.setupLegs()

        # 5th: Check all subscribers work.
        # Get the state of the Robot defined by its RPY orientation, distance from
        # desired point, contact force and JointState of the three joints
        rospy.loginfo("check_all_systems_ready...")
        # self.hexapod_state_object.check_all_systems_ready()
        rospy.loginfo("get_observations...")
        state = self.hexapod_state_object.getObservations()

        # 6th: We restore the gravity to original
        rospy.loginfo("Restore Gravity...")
        self.gazebo.change_gravity(0.0, 0.0, -9.81)
        # 7th: pauses simulation
        # rospy.loginfo("Pause SIM...")
        # self.gazebo.pauseSim()

        #Starts random
        # random_action = np.random.uniform(low=-self.abs_joint_max_action, high=self.abs_joint_max_action, size=(18,))
        random_action = self.action_space.sample()
        pose = self.hexapod_state_object.action2pose(random_action)
        self.hex_controller.setAllLegsAngle(pose)
        time.sleep(self.running_step)
        info = self.hexapod_state_object.getInfo()

        return state,info

    def _render(self, mode='human', close=True):
        pass

    def step(self, action):

        # Given the action selected by the learning algorithm,
        # we perform the corresponding movement of the robot

        # 1st, decide which action corresponsd to which joint is incremented
    

        # We move it to that pos
        pose = self.hexapod_state_object.action2pose(action)
        self.hex_controller.setAllLegsAngle(pose)
        # Then we send the command to the robot and let it go
        # for running_step seconds
        time.sleep(self.running_step)

        # We now process the latest data saved in the class state to calculate
        # the state and the rewards. This way we guarantee that they work
        # with the same exact data.
        # Generate State based on observations
        observation = self.hexapod_state_object.getObservations()

        # finally we get an evaluation based on what happened in the sim
        _reward,done,info= self.hexapod_state_object.processData()
        
        self.reward = _reward - self.cumulated_reward
        self.old_reward = _reward
        self.cumulated_reward += self.reward

        # Get the State Discrete Stringuified version of the observations
        # state = self.get_state(observation)

        return observation, self.reward, done, False,info

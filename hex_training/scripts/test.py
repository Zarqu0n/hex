#! /usr/bin/env python3
from hex_control.legCtrl import LegController
import rospy
import time
from gazebo_connection import GazeboConnection
from hexapod_state import HexapodState
from hexapod_env import HexapodEnv
from controllers_connection import ControllersConnection
from hex_control.hegzi import HexapodController

class Test:
    def __init__(self) -> None:
        self.gazebo = GazeboConnection()
        self.hex_controller = HexapodController()
        self.controllers_object = ControllersConnection(namespace="hexapod")
        self.hexapod_state_object = HexapodState(   abs_max_roll=1,
                                                    abs_max_pitch=1,
                                                    abs_min_pos_dist=0.1,
                                                    abs_max_pos_dist=5,
                                                    abs_max_distance=5)
    def _reset(self):

        # 0st: We pause the Simulator
        rospy.loginfo("Pausing SIM...")
        self.gazebo.pauseSim()

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
        observation = self.hexapod_state_object.getObservations()

        # 6th: We restore the gravity to original
        rospy.loginfo("Restore Gravity...")
        self.gazebo.change_gravity(0.0, 0.0, -9.81)
        # 7th: pauses simulation


if __name__ == "__main__":
    rospy.init_node("hexapod_control_node")
    rospy.loginfo("Starting")
    test = Test()
    test._reset()
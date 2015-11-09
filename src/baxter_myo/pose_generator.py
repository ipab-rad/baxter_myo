import os
from math import radians, degrees, fabs
from copy import deepcopy

import rospy
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
from baxter_interface import Limb


class PoseGenerator(object):

    def __init__(self, limb, mode, step=5):
        self.limb_name = limb
        self.mode = mode
        self._step = step
        self._limb = Limb(self.limb_name)
        self._subscribe()

    def _subscribe(self):
        self._last_data_0 = (Vector3(), Vector3())
        self._last_data_1 = (Vector3(), Vector3())
        self._calib_data_0 = (Vector3(), Vector3())
        self._calib_data_1 = (Vector3(), Vector3())
        self._sub_ori_0 = rospy.Subscriber("/myo_0/orientation",
                                           Vector3,
                                           self._orientation_callback_0)
        self._sub_pos_0 = rospy.Subscriber("/myo_0/position",
                                           Vector3,
                                           self._position_callback_0)
        self._sub_ori_1 = rospy.Subscriber("/myo_1/orientation",
                                           Vector3,
                                           self._orientation_callback_1)
        self._sub_pos_1 = rospy.Subscriber("/myo_1/position",
                                           Vector3,
                                           self._position_callback_1)

    def _orientation_callback_0(self, data):
        self._last_data_0[0].x = data.x
        self._last_data_0[0].y = data.y
        self._last_data_0[0].z = data.z

    def _position_callback_0(self, data):
        self._last_data_0[1].x = data.x
        self._last_data_0[1].y = data.y
        self._last_data_0[1].z = data.z

    def _orientation_callback_1(self, data):
        self._last_data_1[0].x = data.x
        self._last_data_1[0].y = data.y
        self._last_data_1[0].z = data.z

    def _position_callback_1(self, data):
        self._last_data_1[1].x = data.x
        self._last_data_1[1].y = data.y
        self._last_data_1[1].z = data.z

    def calibrate(self):
        """
        Calibrate position of the robot arm wrt the myo data
        """
        raw_input("Press enter when user is at the right pose")
        self._calib_data_0 = deepcopy(self._last_data_0)
        self._calib_data_1 = deepcopy(self._last_data_1)
        self._calib_pose = self._limb.joint_angles()

    def _is_vector_valid(self, data):
        """
        Check if data is likely to be valid.
        """
        test = data.x + data.y + data.z
        return test != 0

    def _is_over_step(self, change):
        return abs(change) > self._step

    def generate_pose(self):
        """
        Given new data and position of the arm, calculate new
        joint positions.
        """

        os.system('clear')
        rospy.loginfo("Generating pose")

        data_0 = deepcopy(self._last_data_0[0])
        data_1 = deepcopy(self._last_data_1[0])
        this_pose = deepcopy(self._calib_pose)

        if self._is_vector_valid(data_0):
            change_0 = Vector3()
            change_0.x = data_0.x - self._calib_data_0[0].x
            change_0.y = data_0.y - self._calib_data_0[0].y
            change_0.z = data_0.z - self._calib_data_0[0].z
            print "MYO_0 (Forearm)"
            print "PITCH: ", change_0.y
            if (self._is_over_step(change_0.y)):
                this_pose["right_w1"] += radians(-1 * change_0.y)
            print "YAW: ", change_0.z
            if (self._is_over_step(change_0.z)):
                this_pose["right_w0"] += radians(-1 * change_0.z)
            print "ROLL: ", change_0.x
            if (self._is_over_step(change_0.x)):
                this_pose["right_w2"] += radians(-1 * change_0.x)

        if self._is_vector_valid(data_1):
            change_1 = Vector3()
            change_1.x = data_1.x - self._calib_data_1[0].x
            change_1.y = data_1.y - self._calib_data_1[0].y
            change_1.z = data_1.z - self._calib_data_1[0].z
            print "MYO_1 (Upper arm)"
            print "PITCH: ", change_1.y
            if (self._is_over_step(change_1.y)):
                this_pose["right_e1"] += radians(change_1.y)
            print "YAW: ", change_1.z
            if (self._is_over_step(change_1.z)):
                this_pose["right_e0"] += radians(change_1.z)
            print "ROLL: ", change_1.x
            if (self._is_over_step(change_1.x)):
                this_pose["right_s0"] += radians(change_1.x)

        return this_pose

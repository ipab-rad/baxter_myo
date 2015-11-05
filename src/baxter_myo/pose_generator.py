import math
from copy import deepcopy

import rospy
import tf
from std_msgs.msg import Header

from geometry_msgs.msg import Vector3

from baxter_interface import Limb

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


class PoseGenerator(object):

    def __init__(self, limb, mode, init_pose):
        self.limb_name = limb
        self.mode = mode
        self.init_pose = init_pose
        self._limb = Limb(self.limb_name)

        # IK service stuff
        self._iksrvp = rospy.ServiceProxy(
            "ExternalTools/" + self.limb_name
            + "/PositionKinematicsNode/IKService",
            SolvePositionIK)
        self._ik_request = SolvePositionIKRequest()
        self._last_ori = Vector3()
        self._last_pos = Vector3()
        self._sub_ori = rospy.Subscriber("/myo_0/orientation",
                                         Vector3,
                                         self._orientation_callback)
        self._sub_pos = rospy.Subscriber("/myo_0/position",
                                         Vector3,
                                         self._position_callback)


    def _orientation_callback(self, data):
        self._last_ori = data

    def _position_callback(self, data):
        self._last_pos = data


    def generate_pose(self):
        """
        Given new data and position of the arm, calculate new
        joint positions.
        """
        rospy.loginfo("Generating pose")
        rospy.loginfo("Position: %s", self._last_pos)
        rospy.loginfo("Orientation: %s", self._last_ori)
        return None

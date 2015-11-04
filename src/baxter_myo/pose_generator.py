import math
from copy import deepcopy

import rospy
import tf
from std_msgs.msg import Header

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Twist,
    Quaternion,
    Vector3
)

from baxter_interface import (
    Limb,
)

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
        # TODO subscribe to topics

    def generate_pose(self, new_data):
        """
        Given new data and position of the arm, calculate new
        joint positions.
        """
        raise(NotImplementedError)

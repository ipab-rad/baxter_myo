import sys
from copy import deepcopy
import time
import math

import rospy
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Twist,
    Quaternion,
)

import baxter_interface
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from baxter_myo.msg import MyoData

class ArmController(object):

    def __init__(self, limb, starting_pos=None, push_thresh=100,
                 mode='positions', rgbd=False):
        rospy.loginfo("Moving to neutral position")
        self.move_to_neutral()
        self.initial_end_pose = {}
        rospy.loginfo("Recording offset")
        self.set_offset()

    def move_to_neutral(self):
        self._limb.move_to_joint_positions(self.neutral_pos)

    def set_offset(self):
        pose = self._limb.endpoint_pose()
        self.initial_end_pose = pose
        eu = tf.transformations.euler_from_quaternion(pose['orientation'])
        self.baxter_off.linear.x = pose['position'][0]
        self.baxter_off.linear.y = pose['position'][1]
        self.baxter_off.linear.z = pose['position'][2]
        self.baxter_off.angular.x = eu[0]
        self.baxter_off.angular.y = eu[1]
        self.baxter_off.angular.z = eu[2]

    def is_pushing(self):
        e = self._limb.joint_efforts()
        s = sum([abs(e[i]) for i in e.keys()])
        return s > self.push_thresh

def main():
    ac = ArmController('right')
    while 1:
        ac.step()

if __name__ == "__main__":
    sys.exit(main())

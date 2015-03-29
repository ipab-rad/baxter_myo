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

from baxter_myo.pose_generator import PoseGenerator
from baxter_myo.msg import MyoData

class ArmController(object):

    def __init__(self, limb, starting_pos=None, push_thresh=100,
                 mode='positions', rgbd=False):
        self.limb_name = limb
        self._limb = baxter_interface.Limb(self.limb_name)
        self._neutral_pos = starting_pos
        self._mode = mode
        rospy.loginfo("Moving to neutral position")
        self.move_to_neutral()
        rospy.loginfo("Initialising PoseGenerator")
        self._pg = PoseGenerator(limb,
                                 self._mode,
                                 self._limb.endpoint_pose())
        # rospy.loginfo("Recording offset")
        # self.set_offset()

    def move_to_neutral(self):
        self._limb.move_to_joint_positions(self._neutral_pos)

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

    def step(self):
        pos = self._pg.gen_next()
        if pos is not None:
            print "Moving to: " + str(pos)
            self._limb.move_to_joint_positions(pos, timeout=0.2)
        else:
            rospy.logwarn("Generated pos is null!")



def main():
    ac = ArmController('right')
    while 1:
        ac.step()

if __name__ == "__main__":
    sys.exit(main())

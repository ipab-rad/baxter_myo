from math import max

import rospy
from baxter_interface import Limb

from baxter_myo.pose_generator import PoseGenerator


class ArmController(object):

    def __init__(self, limb_name, starting_pos=None, push_thresh=10,
                 mode='positions'):
        """
        Initialises parameters and moves the arm to a neutral
        position.
        """
        self.limb_name = limb_name
        self._limb = Limb(self.limb_name)
        self._neutral_pos = starting_pos
        self._mode = mode
        rospy.loginfo("Moving to neutral position")
        self.move_to_neutral()
        rospy.loginfo("Initialising PoseGenerator")
        self._pg = PoseGenerator(self.limb_name,
                                 self._mode,
                                 self._limb.endpoint_pose())

    def move_to_neutral(self):
        self._limb.move_to_joint_positions(self._neutral_pos)

    def is_pushing(self):
        """
        Checks if any of the joints is under external stress. Returns
        true if the maximum recorded stress above specified threshold.
        """
        e = self._limb.joint_efforts()
        max_effort = max([abs(e[i]) for i in e.keys()])
        return max_effort > self.push_thresh

    def step(self):
        """
        Executes a step of the main routine.
        """
        pos = self._pg.gen_next()
        if pos is not None:
            rospy.loginfo("Moving to position %s", pos)
            self._limb.move_to_joint_positions(pos, timeout=0.2)
        else:
            rospy.logwarn("Generated position is invalid")


def main():
    ac = ArmController('right')
    while not rospy.is_shutdown():
        ac.step()

if __name__ == "__main__":
    main()

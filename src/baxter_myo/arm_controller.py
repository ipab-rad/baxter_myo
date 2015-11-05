import rospy
from std_msgs.msg import String
from baxter_interface import Limb, Gripper, CHECK_VERSION

from baxter_myo.pose_generator import PoseGenerator


class ArmController(object):

    def __init__(self, limb_name, starting_pos=None, push_thresh=10,
                 mode='positions'):
        """
        Initialises parameters and moves the arm to a neutral
        position.
        """
        self.limb_name = limb_name
        rospy.loginfo("Creating interface and calibrating gripper")
        self._limb = Limb(self.limb_name)
        self._gripper = Gripper(self.limb_name, CHECK_VERSION)
        self._gripper.calibrate()
        self._is_fist_closed = False

        self._neutral_pos = starting_pos
        self._mode = mode
        rospy.loginfo("Moving to neutral position")
        self.move_to_neutral()
        rospy.loginfo("Initialising PoseGenerator")
        self._pg = PoseGenerator(self.limb_name,
                                 self._mode,
                                 self._limb.endpoint_pose())

        self._sub_gesture = rospy.Subscriber("/myo_0/gesture", String,
                                             self._gesture_callback)
        self._last_data = None

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

    def _command_gripper(self):
        """
        Reads state from Myo and opens/closes gripper as needed.
        """
        if self._gripper.moving():
            return

        if self._is_fist_closed:
            self._gripper.close()
        else:
            self._gripper.open()

    def step(self):
        """
        Executes a step of the main routine.
        Fist checks the status of the gripper and
        """

        self._command_gripper()

        pos = self._pg.generate_pose()

        if pos is not None:
            rospy.loginfo("Moving to position %s", pos)
            if not self.is_pushing():
                self._limb.move_to_joint_positions(pos, timeout=0.2)
            else:
                rospy.logwarn("Arm is being pushed!")
        else:
            rospy.logwarn("Generated position is invalid")

    def _gesture_callback(self, data):
        self._is_fist_closed = (data is "Fist")


def main():
    ac = ArmController('right')
    while not rospy.is_shutdown():
        ac.step()

if __name__ == "__main__":
    main()

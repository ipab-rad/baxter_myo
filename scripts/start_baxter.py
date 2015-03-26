#!/usr/bin/python
import time
import rospy

from baxter_myo.arm_controller import ArmController
from baxter_myo.config_reader import ConfigReader


def main():
    c = ConfigReader("demo_config")
    c.parse_all()
    s = ArmController('right', c.right_angles, c.push_thresh)
    while not rospy.is_shutdown():
        s.step()

if __name__ == "__main__":
    main()

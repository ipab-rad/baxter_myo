#!/usr/bin/python

from baxter_myo.arm_controller import ArmController
from baxter_myo.config_reader import ConfigReader


def main():
    c = ConfigReader("demo_config")
    c.parse_all()
    s = ArmController('right', c.right_angles)
    s.loop()

if __name__ == "__main__":
    main()

#!/usr/bin/python

from baxter_myo.ArmController import ArmController


def main():
    s = ArmController()
    s.loop()

if __name__ == "__main__":
    main()

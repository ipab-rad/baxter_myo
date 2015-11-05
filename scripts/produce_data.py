#!/usr/bin/python
import time
import sys
import random

import rospy
import cv_bridge
import cv
import rospkg

from geometry_msgs.msg import Vector3


class DataTester(object):

    def __init__(self, myo_number, mode="zero"):
        self.mode = mode
        self._myo_name = "myo_" + str(myo_number)
        rospy.init_node("baxter_myo_data_producer")
        self._pub_pos = rospy.Publisher(self._myo_name + "/position",
                                        Vector3,
                                        queue_size=10)
        self._pub_ori = rospy.Publisher(self._myo_name + "/orientation",
                                        Vector3,
                                        queue_size=10)
        self.prt_counter = 0

    def publish(self):
        """
        Publish dummy data
        """
        if self.mode is "zero":
            self._zero_publish()
        elif self.mode is "random":
            self._random_publish()
        else:
            raise Exception("Mode was not recognised")

    def _zero_publish(self):
        msg = Vector3(0, 0, 0)
        self._pub_pos.publish(msg)
        self._pub_ori.publish(msg)
        self._print_dot()


    def _random_publish(self):
        msg = Vector3()
        msg.x = random.randint(-180, 180) # rotation around x (roll)
        msg.y = random.randint(0, 180) # rotation around y (pitch)
        msg.z = random.randint(-180, 180) #rotation around z (yaw)
        self._pub_pos.publish(msg)
        self._pub_ori.publish(msg)
        self._print_dot()

    def _print_dot(self):

        self.prt_counter += 1
        if self.prt_counter > 50:
            print "."
            self.prt_counter = 0
        else:
            print ".",
            sys.stdout.flush()

def main():
    dt = DataTester(0, mode="random")
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        dt.publish()
        r.sleep()

if __name__ == "__main__":
    main()

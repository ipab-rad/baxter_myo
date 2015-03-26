#!/usr/bin/python
import time
import rospy
import cv_bridge
import cv
import rospkg

from baxter_myo.arm_controller import ArmController
from baxter_myo.config_reader import ConfigReader

from sensor_msgs.msg import (
    Image
)


def send_image():
        """
        Send the image located at the specified path to the head
        display on Baxter.
        @param path: path to the image file to load and send
        """
        rp = rospkg.RosPack()
        path = rp.get_path('baxter_myo') \
               + '/share/' + 'good_face.jpg'
        img = cv.LoadImage(path)
        msg = cv_bridge.CvBridge().cv_to_imgmsg(img, encoding="bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
        pub.publish(msg)
        rospy.sleep(1)


def main():
    c = ConfigReader("demo_config")
    c.parse_all()
    s = ArmController('right', c.right_angles, c.push_thresh)
    send_image()
    while not rospy.is_shutdown():
        s.step()

if __name__ == "__main__":
    main()

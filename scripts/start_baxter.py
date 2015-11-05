#!/usr/bin/python
import time
import rospy
import cv_bridge
import cv2
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
        img = cv2.imread(path)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=10)
        pub.publish(msg)
        rospy.sleep(1)

def main():
    rospy.init_node("baxter_myo_controller")
    c = ConfigReader("demo_config")
    c.parse_all()
    s = ArmController('right', c.right_angles, c.push_thresh, c.mode)
    send_image()
    while not rospy.is_shutdown():
        s.step()

if __name__ == "__main__":
    main()

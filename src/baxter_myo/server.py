import socket

import rospy
from geometry_msgs.msg import (
    Twist,
    Vector3
)
from std_msgs.msg import (
    String,
)

from baxter_myo.msg import MyoData

class SocketListener(object):

    def __init__(self, host, port, topic_low=None, topic_high=None):
        # ROS stuff first
        rospy.init_node("myo_socket_listener")
        if topic_low is not None:
            self._pub_low = rospy.Publisher(topic_low, MyoData)
        if topic_high is not None:
            self._pub_high = rospy.Publisher(topic_high, MyoData)
        self.high_calibrated = False
        self.high_enabled = False
        self.high_gripper = False
        self.low_calibrated = True
        self.low_enabled = True
        self.low_gripper = False

        # networking stuff later
        self.host = host
        self.port = port
        self._socket = socket.socket(socket.AF_INET,
                                    socket.SOCK_STREAM)
        self._socket.setsockopt(socket.SOL_SOCKET,
                                socket.SO_REUSEADDR,
                                1)
        self._socket.bind((self.host, self.port))
        self._socket.listen(1)
        self._conn, self.addr = self._socket.accept()
        rospy.loginfo("Connected by %s", self.addr)

    def loop(self):
        while 1:
            data = self._conn.recv(1024)
            s = repr(data)
            if not data:
                break
            s = s[1:-1] # remove `'`s
            rospy.loginfo("Received: %s", s)
            l = s.split(';')
            l = [x for x in l if x]
            for e in l:
                i = e.split(":")
                if i[0] == "0":
                    msg = self.low_prepare_data(i[1])
                    self._pub_low.publish(msg)
                elif i[0] == "1":
                    msg = self.high_prepare_data(i[1])
                    self._pub_high.publish(msg)
                self._conn.sendall(data)
        self._conn.close()


    def prepare_twist(self, s):
        """
        Returns empty Twist if s == ''
        """
        tw = Twist()
        if len(s) == 0:
            return tw
        data = s.split(" ")
        fs = []
        for d in data:
            fs.append(float(d))
        tw.linear.x = fs[0]
        tw.linear.y = fs[1]
        tw.linear.z = fs[2]
        tw.angular.x = fs[3]
        tw.angular.y = fs[4]
        tw.angular.z = fs[5]
        return tw

    def high_prepare_data(self, s):
        msg = MyoData()
        tw = self.prepare_twist('')

        if s == "calibrated":
            self.high_calibrated = True
        elif s == "open gripper":
            self.high_gripper = False
        elif s == "close gripper":
            self.high_gripper = True
        elif s == "enable":
            self.high_enabled = True
        elif s == "disable":
            self.high_enabled = False
        else:
            tw = self.prepare_twist(s)
        msg.data = tw
        msg.enabled = self.high_enabled
        msg.calibrated = self.high_calibrated
        msg.gripper = self.high_gripper
        return msg

    def low_prepare_data(self, s):
        msg = MyoData()
        tw = self.prepare_twist('')

        if s == "calibrated":
            self.low_calibrated = True
        elif s == "open gripper":
            self.low_gripper = False
        elif s == "close gripper":
            self.low_gripper = True
        elif s == "enable":
            self.low_enabled = True
        elif s == "disable":
            self.low_enabled = False
        else:
            tw = self.prepare_twist(s)
        msg.data = tw
        msg.enabled = self.low_enabled
        msg.calibrated = self.low_calibrated
        msg.gripper = self.low_gripper
        return msg


def main():
    s = SocketListener('', 50007, 'myo_data_low', 'myo_data_high')
    s.loop()

if __name__ == "__main__":
    main()

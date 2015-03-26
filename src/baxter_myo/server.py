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

    def __init__(self, host, port, topic):
        # ROS stuff first
        rospy.init_node("myo_socket_listener")
        self._pub = rospy.Publisher(topic, MyoData)
        self.calibrated = False
        self.enabled = False
        self.gripper = False
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
            rospy.loginfo("Received: %s", s)
            msg = self.prepare_data(s)
            self._pub.publish(msg)
            self._conn.sendall(data)
        self._conn.close()


    def prepare_twist(self, s):
        """
        Returns empty Twist if s == ''
        """
        tw = Twist()
        if len(s) > 0:
            s = s[1:]
            s = s[:-1]
            data = s.split()
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

    def prepare_data(self, s):
        msg = MyoData()
        tw = self.prepare_twist('')

        if s == "calibrated":
            self.calibrated = True
        elif s == "open_gripper":
            self.gripper = False
        elif s == "close_gripper":
            self.gripper = True
        elif s == "enable":
            self.enabled = True
        elif s == "disable":
            self.enabled = False
        else:
            tw = self.prepare_twist(s)

        msg.data = tw
        msg.enabled = self.enabled
        msg.calibrated = self.calibrated
        msg.gripper = self.gripper
        return msg


def main():
    s = SocketListener('', 50007, 'myo_data')
    s.loop()

if __name__ == "__main__":
    main()

#!/usr/bin/python

import socket

import rospy
from geometry_msgs.msg import (
    Twist
)
from std_msgs.msg import (
    String,
)

class SocketListener(object):

    def __init__(self, host, port, topic):
        # ROS stuff first
        rospy.init_node("myo_socket_listener")
        self._pub = rospy.Publisher(topic, String)

        # networking stuff later
        self.host = host
        self.port = port
        self._socket = socket.socket(socket.AF_INET,
                                    socket.SOCK_STREAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
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
            self._pub.publish(s)
            self._conn.sendall(data)
        self._conn.close()


def main():
    s = SocketListener('', 50007, 'myo_data')
    s.loop()

if __name__ == "__main__":
    main()

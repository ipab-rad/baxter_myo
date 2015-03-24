import socket

import rospy
from geometry_msgs.msg import (
    Twist,
    Vector3
)
from std_msgs.msg import (
    String,
)

class SocketListener(object):

    def __init__(self, host, port, topic):
        # ROS stuff first
        rospy.init_node("myo_socket_listener")
        self._pub = rospy.Publisher(topic, Twist)
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
            tw = self.prepare_data(s)
            self._pub.publish(tw)
            self._conn.sendall(data)
        self._conn.close()

    def prepare_data(self, s):
        s = s[1:]
        s = s[:-1]
        data = s.split()
        fs = []
        for d in data:
            fs.append(float(d))
        tw = Twist()
        tw.linear.x = fs[0]
        tw.linear.y = fs[1]
        tw.linear.z = fs[2]
        tw.angular.x = fs[3]
        tw.angular.y = fs[4]
        tw.angular.z = fs[5]
        return tw


def main():
    s = SocketListener('', 50007, 'myo_data')
    s.loop()

if __name__ == "__main__":
    main()

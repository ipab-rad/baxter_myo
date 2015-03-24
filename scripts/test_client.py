#!/usr/bin/python

import socket
import time
import random
import sys
HOST = 'windcharger.local'    # The remote host
PORT = 50007              # The same port as used by the server
print("++++++++++++++++ Opening socket ++++++++++++++++")
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
while 1:
    try:
        x = random.uniform(-0.2, 0.2)
        y = random.uniform(-0.2, 0.2)
        z = random.uniform(-0.2, 0.2)
        o_x = random.uniform(0.0, 0.0)
        o_y = random.uniform(-0.0, -0.0)
        o_z = random.uniform(-0.0, 0.0)
        # could just use joint, but *hack*
        st = "" + str(x) + " " + str(y) + " " + str(z) \
             +  " " + str(o_x) + " " + str(o_y) + " " + str(o_z)
        print "Sending:", st
        s.sendall(st)
        data = s.recv(1024)
        time.sleep(1)
    except KeyboardInterrupt:
        s.close()
        print("\n++++++++++ Socket succesfully closed! ++++++++++")
        sys.exit(0)

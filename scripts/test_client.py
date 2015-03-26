#!/usr/bin/python

import socket
import time
import random
import sys

import rospkg

HOST = 'windcharger.local'    # The remote host
PORT = 50007              # The same port as used by the server
print("++++++++++++++++ Opening socket ++++++++++++++++")
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

def parse_doc():
    l = []
    rp = rospkg.RosPack()
    config_path = rp.get_path('baxter_myo') \
                  + '/tests/' + 'poss.txt'
    f = open(config_path, 'r')
    for line in f:
        s = line.split(" ")
        if s[0] == "position":
            x = eval(s[1])
            y = eval(s[2])
            z = eval(s[3])
            l.append((x, y, z))
    return l

while 1:
    try:
        l = parse_doc()
        for e in l:
            x = e[0]
            y = e[1]
            z = e[2]
            o_x = random.uniform(0.0, 0.0)
            o_y = random.uniform(-0.0, -0.0)
            o_z = random.uniform(-0.0, 0.0)
            # could just use joint, but *hack*
            st = "" + str(x) + " " + str(y) + " " + str(z) \
                 +  " " + str(o_x) + " " + str(o_y) + " " + str(o_z) + ";"
            print "Sending:", st
            s.sendall(st)
            data = s.recv(1024)
            time.sleep(0.05)
        break
    except KeyboardInterrupt:
        s.close()
        print("\n++++++++++ Socket succesfully closed! ++++++++++")
        sys.exit(0)

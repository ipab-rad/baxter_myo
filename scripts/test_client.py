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
                  + '/tests/' + 'angles.txt'
    f = open(config_path, 'r')
    for line in f:
        print line
        s = line.split(" ")
        if s[0] == "orientation":
            x = random.uniform(0.0, 0.0)
            y = random.uniform(0.0, 0.0)
            z = random.uniform(0.0, 0.0)
            o_x = eval(s[1])
            o_y = eval(s[2])
            o_z = eval(s[3])
            l.append((x, y, z, o_x, o_y, o_z))
    return l

while 1:
    try:
        l = parse_doc()
        print l
        while 1:
            for e in l:
                x = e[0]
                y = e[1]
                z = e[2]
                o_x = e[3]
                o_y = e[4]
                o_z = e[5]
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

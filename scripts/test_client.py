
# Echo client program
import socket
import time
import random
import sys
HOST = 'windcharger.local'    # The remote host
PORT = 50007              # The same port as used by the server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
while 1:
    try:
        x = float(random.randint(1, 360))
        y = float(random.randint(1, 360))
        z = float(random.randint(1, 360))
        st = "" + str(x) + " " + str(y) + " " + str(z) \
             +  " " + str(x) + " " + str(y) + " " + str(z)
        print "Sending:", st
        s.sendall(st)
        data = s.recv(1024)

        time.sleep(1)
    except KeyboardInterrupt:
        s.close()
        print("Socket succesfully closed")
        sys.exit(0)

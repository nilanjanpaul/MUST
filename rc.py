#!/usr/bin/env python

#Usage: ./rc.py "cmd"
# >     ./rc.py "set rxfreq 2450e6"
# >     ./rc.py "get rxfreq"
#

# Example commands:
# python rc.py node1-1 "rxfreq all 2400e6"
# python rc.py node1-1 "rxfreq all"
# python rc.py node1-1 "txfreq all 2400e6"
# python rc.py node1-1 "txfreq all"
# python rc.py node1-1 "txgain all 19"
# python rc.py node1-1 "txgain all"

# python rc.py node1-1 "rxhold on"
# python rc.py node1-1 "rxhold off"

import sys
import socket
import struct
from time import time

#TCP_IP = sys.argv[1]
#TCP_PORT = int(sys.argv[2])
TCP_IP = sys.argv[1]  #"node1-1.sb3.orbit-lab.org"
TCP_PORT = int(5180)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))

COMMAND =  sys.argv[2]
s.sendall(COMMAND)
data = s.recv(256)

print data

s.close()

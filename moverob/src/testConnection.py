#!/usr/bin/env python

import qi
import naoqi
from myConnection import IP, PORT, RobotConnection
#no diffference between AI and human is our empathic ability...

ROBOIP = IP
ROBOPORT=PORT

print("ROBOT IP:", ROBOIP)
print("ROBOT PORT:", PORT)

session = RobotConnection(ROBOIP, ROBOPORT)
motion_service = session.service("ALMotion")
print(motion_service)

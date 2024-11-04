#!/usr/bin/env python

import io
import qi
import csv
import cv2
import time
import vrep
import math
import rospy
import motion
import numpy as np
import matplotlib.pyplot as plt

from naoqi import ALProxy
from random import randrange
from numpy.linalg import inv, norm
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Twist, Pose 
from sensor_msgs.msg import Image, JointState
from scipy.interpolate import BSpline, make_interp_spline



def RobotConnection(IP, PORT):
    session = qi.Session()
    try:
       session.connect("tcp://" + IP + ":" + str(PORT))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + IP  + "\" on port " + str(PORT) +".\n" "Please check your script arguments. Run with -h option for help.")
    return session


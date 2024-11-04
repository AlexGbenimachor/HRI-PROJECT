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
#from myConnection import IP, PORT, RobotConnection
from scipy.interpolate import BSpline, make_interp_spline


# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError

'''
def myConnection(IP, PORT):
    session = qi.Session()
    try:
       session.connect("tcp://" + IP + ":" + str(PORT))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + IP  + "\" on port " + str(PORT) +".\n" "Please check your script arguments. Run with -h option for help.")
    return session
'''

    
'''
session = RobotConnection(IP, PORT)
#define motiion service...

motion_service = session.service("ALMotion")
motion_service.wakeUp()
'''





def direction_left(motion_service):
    speed = 0.6 #define the speed...

    JointNames = ["RHand", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll",  "RWristYaw"]
    JointNames2 = ["LHand", "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll",  "LWristYaw"]
    jointAngles2 = [90, 90, -30, -45, -45, 0 ]
    jointAngles2 = [x * motion.TO_RAD for x in jointAngles2]
    jointAngles = [45, 20, 45, 0,90, 90 ]
    jointAngles = [x * motion.TO_RAD for x in jointAngles]
    #print(jointAngles)
    #print(jointAngles2)
    motion_service.angleInterpolationWithSpeed(JointNames, jointAngles, speed)
    motion_service.angleInterpolationWithSpeed(JointNames2, jointAngles2, speed)
    
def direction_right(motion_service):
    speed = 0.6 #define the speed...

    JointNames = ["RHand", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll",  "RWristYaw"]
    JointNames2 = ["LHand", "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll",  "LWristYaw"]
    jointAngles2 = [90, 90, -30, -45, -45, 0 ]
    jointAngles2 = [x * motion.TO_RAD for x in jointAngles2]
    jointAngles = [90, 20, -45, 90,0, 0 ]
    jointAngles = [x * motion.TO_RAD for x in jointAngles]
    '''print(jointAngles)
    print(jointAngles2)'''
    motion_service.angleInterpolationWithSpeed(JointNames, jointAngles, speed)
    motion_service.angleInterpolationWithSpeed(JointNames2, jointAngles2, speed)
   

def go_forward(motion_service):
    speed = 0.6 #define the speed...

    jointNames=["LElbowRoll", "LShoulderPitch"]
    jointvalues = [-0.79, 0.5]
    #elbowangles = randrange(len(LElbowValues))
    motion_service.angleInterpolationWithSpeed(jointNames,jointvalues, 0.6)
    jointvalues2 = [0.79, 1]
    motion_service.angleInterpolationWithSpeed(jointNames,jointvalues2, 0.6)



 
def Make_Gesture(motion_service):
    #angles...
    HeadYawVals = [0.1, -0.1]
    HeadPitchVals = [0.1]

    LWristYawVals = [-1.0]
    LElbowRollVals = [-0.7, -1.4]
    LShoulderRollVals = [0.1, 0.5]
    LShoulderPitchVals = [1.2]

    RWristYawVals = [1.4]
    RElbowRollVals = [1.4, 0.7]
    RShoulderRollVals = [-0.1, -0.5]
    RShoulderPitchVals = [1.2]
    
    #joint names
    jointNames = [
        "HeadPitch", 
        "HeadYaw", 
        "LElbowRoll", 
        "LWristYaw", 
        "RElbowRoll", 
        "RWristYaw", 
        "RShoulderRoll", 
        "LShoulderRoll",
        "RShoulderPitch",
        "LShoulderPitch"
    ]
    times = [1.0] * 10

    angles = [
        HeadPitchVals[randrange(len(HeadPitchVals))], 
        HeadYawVals[randrange(len(HeadYawVals))], 
        LElbowRollVals[randrange(len(LElbowRollVals))], 
        LWristYawVals[randrange(len(LWristYawVals))], 
        RElbowRollVals[randrange(len(RElbowRollVals))], 
        RWristYawVals[randrange(len(RWristYawVals))], 
        RShoulderRollVals[randrange(len(RShoulderRollVals))], 
        LShoulderRollVals[randrange(len(LShoulderRollVals))],
        RShoulderPitchVals[randrange(len(RShoulderPitchVals))],
        LShoulderPitchVals[randrange(len(LShoulderPitchVals))] ]
    isAbsolute = True
    motion_service.angleInterpolation(jointNames, angles, times, isAbsolute)


def Hello_Gesture(motion_service):
    #angles...
    HeadYawVals = [0.0]
    HeadPitchVals = [-0.2]

    LWristYawVals = [0.0]
    LElbowRollVals = [-0.5]
    LShoulderRollVals = [0.1]
    LShoulderPitchVals = [1.6]

    RWristYawVals = [0.2]
    RElbowRollVals = [1.5]
    RShoulderRollVals = [-1.0]
    RShoulderPitchVals = [0.2]
    
    #joint names
    jointNames = [
        "HeadPitch", 
        "HeadYaw", 
        "LElbowRoll", 
        "LWristYaw", 
        "RElbowRoll", 
        "RWristYaw", 
        "RShoulderRoll", 
        "LShoulderRoll",
        "RShoulderPitch",
        "LShoulderPitch"
    ]
    times = [1.0] * 10

    angles = [
        HeadPitchVals[randrange(len(HeadPitchVals))], 
        HeadYawVals[randrange(len(HeadYawVals))], 
        LElbowRollVals[randrange(len(LElbowRollVals))], 
        LWristYawVals[randrange(len(LWristYawVals))], 
        RElbowRollVals[randrange(len(RElbowRollVals))], 
        RWristYawVals[randrange(len(RWristYawVals))], 
        RShoulderRollVals[randrange(len(RShoulderRollVals))], 
        LShoulderRollVals[randrange(len(LShoulderRollVals))],
        RShoulderPitchVals[randrange(len(RShoulderPitchVals))],
        LShoulderPitchVals[randrange(len(LShoulderPitchVals))] ]
    isAbsolute = True
    motion_service.angleInterpolation(jointNames, angles, times, isAbsolute)



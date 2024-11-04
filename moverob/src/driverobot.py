#!/usr/bin/env python
import io
import os
import re
import qi
import csv
import time
import math
import vrep
import rospy
import naoqi
import random
import pygame
import pyttsx3
import numpy as np
import simplejson as json
import speech_recognition as sr
import matplotlib.pyplot as plt


from gtts import gTTS 
from naoqi import ALProxy
from random import randrange
from playsound import playsound
from nav_msgs.msg import Odometry
from numpy.linalg import inv, norm
from std_msgs.msg import Int32, Float32
from nltk.tokenize import word_tokenize 
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Image, JointState
from scipy.interpolate import BSpline, make_interp_spline
from myConnection import IP, PORT, RobotConnection
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError

#Drive the Choregraphe...
#initialize the Python Speech Text
engine = pyttsx3.init() # object creation
session = RobotConnection(IP, PORT) #let make a connection
motion_service = session.service("ALMotion")
motion_service.wakeUp()
tts_service = session.service("ALTextToSpeech")
    

#print("let's start the program")
vrep.simxFinish(-1)
#robotics page 537 (motion planning)

clientID = vrep.simxStart('127.0.0.1',19999,True,True,5000,5)


if (clientID != -1):
   print("connected to the vrep server")
else:
   print("it's not connected to the vrep server")
   
   

#get the objects on the simulator...
Dwheel = (2.0000e-02+2.0000e-01+2.0000e-02)  #define the wheel base...


#buildings in the Vrep Simulation...
clientID, office = vrep.simxGetObjectHandle(clientID, "office", vrep.simx_opmode_blocking )
clientID, library = vrep.simxGetObjectHandle(clientID, "library", vrep.simx_opmode_blocking )
clientID, cafeteria = vrep.simxGetObjectHandle(clientID, "cafeteria", vrep.simx_opmode_blocking )
clientID, classroom = vrep.simxGetObjectHandle(clientID, "classroom", vrep.simx_opmode_blocking )
clientID, defPosition = vrep.simxGetObjectHandle(clientID, "defpos", vrep.simx_opmode_blocking )


#get Motors...
clientID, rightMotor = vrep.simxGetObjectHandle(clientID, "rightMotor", vrep.simx_opmode_blocking )
clientID, leftMotor = vrep.simxGetObjectHandle(clientID, "leftMotor", vrep.simx_opmode_blocking )
clientID, bubbleSlider = vrep.simxGetObjectHandle(clientID, "bubbleRob_slider", vrep.simx_opmode_blocking )
clientID, bubbleRob = vrep.simxGetObjectHandle(clientID, "bubbleRob", vrep.simx_opmode_blocking )


#get Sensors...
clientID, bubbleEyes = vrep.simxGetObjectHandle(clientID, "bubbleRob_eyes", vrep.simx_opmode_blocking )
clientID, bubbleConnection = vrep.simxGetObjectHandle(clientID, "bubbleRob_connection", vrep.simx_opmode_blocking )
clientID, RobbyNose = vrep.simxGetObjectHandle(clientID, "bubbleRob_SensingNose", vrep.simx_opmode_blocking )
#print Objects...
print("\n")
print("Buildings")
print("____"*10)
#print building...
print("office building:", office)
print("library building:", library)
print("cafeteria building:", cafeteria)
print("classroom building:", classroom)
print("default Position:", defPosition)
   
print("\n")
print("Motors Objects")
print("____"*10)
#print Motors...
print("right Object:", rightMotor)
print("left Object:",  leftMotor)
print("slider:", bubbleSlider)
print("bubbleRob:", bubbleRob)

print("\n")
#print Sensors
print("Sensors")
print("____"*10)
print("RobbyNose:",    RobbyNose)
print("Bubble Eyes:", bubbleEyes)
print("connection:", bubbleConnection)

print("\n")
#get the building location...
print("building Location")
print("____"*10)
returnCode, officePos =  vrep.simxGetObjectPosition(clientID, office, -1, vrep.simx_opmode_blocking)
returnCode, libraryPos =  vrep.simxGetObjectPosition(clientID, library, -1, vrep.simx_opmode_blocking)
returnCode, cafeteriaPos =  vrep.simxGetObjectPosition(clientID, cafeteria, -1, vrep.simx_opmode_blocking)
returnCode, classroomPos =  vrep.simxGetObjectPosition(clientID, classroom, -1, vrep.simx_opmode_blocking)
returnCode, defaultPos = vrep.simxGetObjectPosition(clientID,  defPosition, -1, vrep.simx_opmode_blocking)
    
   

print("\n")
#print robot location...
print("robot location")
print("____"*10)
returnCode, robotPos = vrep.simxGetObjectPosition(clientID,  bubbleRob, -1, vrep.simx_opmode_blocking)
print("\n")
#print robot position...
print("robot position")
print("bubble Rob:", robotPos)


print("\n")
#print building position...
print("building position")
print("____"*10)
print("office position:", officePos)
print("library position:", libraryPos)
print("cafeteria position:", cafeteriaPos)
print("classroom position:", classroomPos)
print("Default Position:", defaultPos)

#pepper Movements function...



#robot position...
def getRobotPos():
    returnCode, robotPos = vrep.simxGetObjectPosition(clientID,  bubbleRob, -1, vrep.simx_opmode_blocking)
    theta = math.atan2(round(robotPos[1],2), round(robotPos[0],2))
    return[round(robotPos[0],2), round(robotPos[1],2), round(theta,2)]
    

#location positions...    
def getOfficePos():
    returnCode, officePos =  vrep.simxGetObjectPosition(clientID, office, -1, vrep.simx_opmode_blocking)
    theta = math.atan2(officePos[1], officePos[0])
    return [round(officePos[0],2), round(officePos[1],2), round(theta,2)]
        

def getCafeteriaPos():
    returnCode, cafeteriaPos =  vrep.simxGetObjectPosition(clientID, cafeteria, -1, vrep.simx_opmode_blocking)
    theta = math.atan2(cafeteriaPos[1], cafeteriaPos[0])
    return [round(cafeteriaPos[0],2), round(cafeteriaPos[1],2), round(theta,2)]
    
def getLibraryPos():
    returnCode, libraryPos =  vrep.simxGetObjectPosition(clientID, library, -1, vrep.simx_opmode_blocking)
    theta = math.atan2(libraryPos[1], libraryPos[0])
    return [round(libraryPos[0],2), round(libraryPos[1],2), round(theta,3)]
    
def classroomPos():
    returnCode, classroomPos =  vrep.simxGetObjectPosition(clientID, classroom, -1, vrep.simx_opmode_blocking)
    theta = math.atan2(classroomPos[1], classroomPos[0])
    return[round(classroomPos[0],2), round(classroomPos[1]), round(theta,2)]
    
    
def getDefaultPosition():
    returnCode, defaultPos = vrep.simxGetObjectPosition(clientID,  defPosition, -1, vrep.simx_opmode_blocking)
    theta = math.atan2(defaultPos[1], defaultPos[0])
    return[round(defaultPos[0],2), round(defaultPos[1],2), round(theta,2)]  





"""drive Robot..."""
def move_forward(motion_service):
    r = 2
    #print("move forward:", r)
    lv =  0.5*r
    av = 0.2
    dt = abs(lv/av)
    motion_service.move(lv, 0, 0)
    #time.sleep(dt)
    #motionService.stopMove()
    
    
    
def move_backward(motion_service):
    r = -2
    #print("move forward:", r)
    lv =  0.5*r
    av = 0.2
    dt = abs(lv/av)
    motion_service.move(lv, 0, 0)
    #time.sleep(dt)
    #motionService.stopMove()
    
    
def move_right(motion_service):
    r = 1
    #print("turn right:", r)
    angle = math.pi/2
    dt = abs(angle)
    #print("dt:", dt)
    motion_service.move(0, 0, -angle)
    #time.sleep(dt)
   # motionService.stopMove()
       
    

def move_left(motion_service):
    r = 1
    #print("turn right:", r)
    angle = math.pi/2
    dt = abs(angle)
    #print("dt:", dt)
    motion_service.move(0, 0, angle)
    #time.sleep(dt)
    #motionService.stopMove()
            
     
def move_round(motion_service):
     C = 2 * math.pi * 0.07#compute the circumference...
     C = round(C, 2)
     dwheel = 0.176
     angleToRAD =  360 * math.pi/180
     #compute the rotation...
     rotation  = round(angleToRAD,2) * dwheel/(2*C)
     V = 0.3
     W = V/0.07
     d = 2 #steps...
     dt = d/V
     motion_service.move(0,W, round(rotation,2))
     #time.sleep(dt)
     #motionService.stopMove() 

def playaudio(my_text):
    #global engine
    
    engine.setProperty('voice', 'english+f1')
    engine.say(my_text)
    engine.runAndWait()
    time.sleep(1)


#drive Robot Forward, Backward, turnleft, turn right VREP..
def forward():
    print("Pepper:", "we are moving forward")
    #gTTS(text = "we are moving forward", lang='en', slow=False)
    #playaudio("we are moving forward")
    returnCode = vrep.simxSetJointTargetVelocity(clientID, leftMotor, 1, vrep.simx_opmode_blocking) 
    returnCode = vrep.simxSetJointTargetVelocity(clientID, rightMotor,1, vrep.simx_opmode_blocking )
    move_forward(motion_service)
    '''
    
    sayForward = gTTS(text = "we are moving forward", lang='en', slow=False)
    sayForward.save("straight.mp3")
    playsound("straight.mp3")
    tts_service.say("we are moving forward")
    '''
def backward():
    print("Pepper:", "we are moving backward")
    #playaudio("we are moving backward")
    returnCode = vrep.simxSetJointTargetVelocity(clientID, leftMotor, -1, vrep.simx_opmode_blocking) 
    returnCode = vrep.simxSetJointTargetVelocity(clientID, rightMotor,-1, vrep.simx_opmode_blocking)
    move_backward(motion_service)
    '''
   
    sayForward.save("backward.mp3")
    playsound("backward.mp3")
    tts_service.say("we are moving backward") 
    '''

def turnLeft(O_left,  O_right ):
    print("Pepper:", "we will turn left")
    #playaudio("we will turn left")
    returnCode = vrep.simxSetJointTargetVelocity(clientID, leftMotor, O_left, vrep.simx_opmode_blocking ) 
    returnCode = vrep.simxSetJointTargetVelocity(clientID, rightMotor, O_right, vrep.simx_opmode_blocking) 
    move_left(motion_service)
    '''
    print("Pepper:", "we will turn left")
    sayRight = gTTS(text = "we will turn left", lang='en', slow=False)
    sayRight.save("left.mp3")
    playsound("left.mp3")
    tts_service.say("we will turn left")
    '''
    

def turnRight(O_left,  O_right):
    print("Pepper:", "we will turn right")
    #playaudio("we will turn right")
    returnCode = vrep.simxSetJointTargetVelocity(clientID, leftMotor, O_left,vrep.simx_opmode_blocking ) 
    returnCode = vrep.simxSetJointTargetVelocity(clientID, rightMotor, O_right, vrep.simx_opmode_blocking) 
    move_right(motion_service)
    '''
    print("Pepper:", "we will turn right")
    sayRight = gTTS(text = "we will turn right", lang='en', slow=False)
    sayRight.save("right.mp3")
    playsound("right.mp3")
    tts_service.say("we will turn right")
    '''

def stop():
    #print("Pepper:", "please we've arrived to your destination")
    #playaudio("please we've arrived to your destination")
    returnCode = vrep.simxSetJointTargetVelocity(clientID, rightMotor,0, vrep.simx_opmode_blocking) 
    returnCode = vrep.simxSetJointTargetVelocity(clientID, leftMotor, 0, vrep.simx_opmode_blocking) 
    motion_service.stopMove()
    '''
    print("Pepper:", "please we are now in front of the office")
    sayInfront = gTTS(text = "please we are now in front of the office", lang='en', slow=False) 
    sayInfront.save("front.mp3")
    playsound("front.mp3")
    tts_service.say("please we are  now in front of the office")
    '''
 


#Detect Object sensors

def ObjectDetect():
    returnCode,detected, detectObjectPoint, detectedObjectHandle,NormVector=vrep.simxReadProximitySensor(clientID, RobbyNose,vrep.simx_opmode_oneshot_wait)
    return [detected, detectObjectPoint, detectedObjectHandle]
       
       

     

'''
print("robot position  data:", robdata) 
print("office position data:", officeData)
print("library position data:", libraryData)
print("cafeteria position data:", cafeteriaData)
print("classroom position data:", classroomData)  
'''

def distance_and_angle(robData, locData, inDegrees = False):
    #compute the distance...
    distance = math.sqrt(((robData[0]-locData[0] )**2) + ((robData[1]-locData[1])**2))
    
    #compute the radian goal angle...
    radian_angle = math.atan2(locData[1]-robData[1], locData[0]-robData[0])
    
    deg_angle = np.degrees(radian_angle)
    if inDegrees:
       return [round(distance,2), round(deg_angle,2)]
    else:
       return[round(distance,2), round(radian_angle,2)]
       
#compute the radius...       
def computeRadius():
    radius = math.sqrt((0.08)**2 + (0.08)**2)
    radius = round(radius,3)
    return radius
    
#compute rotation...
def computeRotation(angle):
    radius = computeRadius()
    Cir = 2 * math.pi * radius
    rotation = angle * Dwheel/Cir*2
    return rotation
    
    
    
#flag = True

def GoToOffice():
  global office
  #global flag
  flag = True 
  ControlTurn = 0
  while flag:
     detected  = ObjectDetect()
     robdata = getRobotPos() 
     officeData = getOfficePos()
     #print("location:", office)
     #print("detected results:", detected)  
    # print("robot position  data:", robdata) 
     #print("office position data:", officeData)
     distangle = distance_and_angle(robdata,  officeData, inDegrees = False)
     radius = computeRadius()
     distance = distangle[0]
     angle = distangle[1]
     #print("distance:", distance)
     #print("distance and angle:", distangle)
     #print("distance wheel:", Dwheel)  
     #print("radius:", radius)
     targetRotation = computeRotation(angle)
     #print("new angle:", targetRotation)
     #V_linear, O_angular, VR => Speed Right, VL => Speed Right 
     V_rob = 0.1
     O_rob = 0.1 * targetRotation
     VR = V_rob + Dwheel/2 * O_rob
     VL = V_rob - Dwheel/2 * O_rob
     #print("Velocity Left:", VL)
     #print("Velocity Right:", VR)
     #compute the omega ...
     Omega_Left = VL/radius
     Omega_Left = round(Omega_Left,2)
     Omega_Right = VR/radius
     Omega_Right = round(Omega_Right,2) 
     #print("Omega Left:", Omega_Left)
     #print("Omega Right:", Omega_Right)

     #drive Robot...
     
     if (round(targetRotation,2) > round(ControlTurn,2)):
        turnLeft(Omega_Left,  0 )
        ControlTurn = round(ControlTurn,2) + 0.02
        #print("ControlTurn Left:", ControlTurn)
       
        if (round(targetRotation,2) == round(ControlTurn,2)):
           time.sleep(1)
        forward()
     elif  (round(targetRotation,2) < round(ControlTurn,2)):
        turnRight(0,  Omega_Right)
        ControlTurn = round(ControlTurn,2) - 0.02
        #print("ControlTurn Right:", ControlTurn)  
        if (round(targetRotation,2) == round(ControlTurn,2)):
          time.sleep(1)
        forward()
     elif detected[0] == True and detected[2] != office:
          #print("if non target object detected")
          backward()
          time.sleep(1)
          detcoord = detected[1]
          detangle = math.atan2(detcoord[1], detcoord[0])
          devAngle = (angle - detangle)# + 10
          devRot   = computeRotation(devAngle)#compute the deviation angle
          v_dev = 1.0
          o_dev = 1.0 * devRot
          vr_dev = v_dev +  Dwheel/2 * o_dev
          vl_dev = v_dev -  Dwheel/2 * o_dev
          #print("Velocity Deviation Left:",  vl_dev)
          #print("Velocity Deviation Right:", vr_dev)
          dv_omegaLeft = vl_dev/radius
          dv_omegaLeft = round(dv_omegaLeft, 2)
          dv_omegaRight= vr_dev/radius
          dv_omegaRight= round(dv_omegaRight,2)
          #print("Omega Deviation Left:", dv_omegaLeft)
          #print("Omega Deviation Right:",dv_omegaRight)
          if (round(devRot,2) > round(ControlTurn,2)):
             turnLeft(dv_omegaLeft,  0 )
             ControlTurn = round(ControlTurn,2) + 0.02
             if (round(devRot,2) == round(ControlTurn,2)):
                time.sleep(1)
             backward()
             continue
          elif (round(devRot,2) < round(ControlTurn,2)):
             turnRight(0,   dv_omegaRight )
             ControlTurn = round(ControlTurn,2) + 0.02
             if (round(devRot,2) == round(ControlTurn,2)):
                time.sleep(1)
             backward()
             continue 
     if round(distance) <=1.23: #and detected[2] == office:
        '''print("Pepper: we are at the office")
        sayInfront = gTTS(text = "we are at the office", lang='en', slow=False) 
        sayInfront.save("front.mp3")
        playsound("front.mp3")'''
        flag = False
        stop()
        #break
       
        
        #break
   



def GoToLibrary():
  #global flag
  flag = True
  global library
  ControlTurn = 0
  while flag:
     detected  = ObjectDetect()
     robdata = getRobotPos() 
     libraryData = getLibraryPos()  
     #print("location:", library)
     #print("detected results:", detected) 
     #print("robot position  data:", robdata) 
     #print("office position data:", libraryData)
     distangle = distance_and_angle(robdata,  libraryData, inDegrees = False)
     radius = computeRadius()
     distance = distangle[0]
     angle = distangle[1]
     #print("distance:", distance)
     #print("distance and angle:", distangle)
     #print("distance wheel:", Dwheel)  
     #print("radius:", radius)
     targetRotation = computeRotation(angle)
     #print("new angle:", targetRotation)
     #V_linear, O_angular, VR => Speed Right, VL => Speed Right 
     V_rob = 0.1
     O_rob = 0.1 * targetRotation
     VR = V_rob + Dwheel/2 * O_rob
     VL = V_rob - Dwheel/2 * O_rob
     #print("Velocity Left:", VL)
     #print("Velocity Right:", VR)
     #compute the omega ...
     Omega_Left = VL/radius
     Omega_Left = round(Omega_Left,2)
     Omega_Right = VR/radius
     Omega_Right = round(Omega_Right,2) 
     #print("Omega Left:", Omega_Left)
     #print("Omega Right:", Omega_Right)

     #drive Robot...
     
     if (round(targetRotation,2) > round(ControlTurn,2)):
        turnLeft(Omega_Left,  0 )
        ControlTurn = round(ControlTurn,2) + 0.02
        #print("ControlTurn Left:", ControlTurn)
       
        if (round(targetRotation,2) == round(ControlTurn,2)):
           time.sleep(1)
        forward()
     elif  (round(targetRotation,2) < round(ControlTurn,2)):
        turnRight(0,  Omega_Right)
        ControlTurn = round(ControlTurn,2) - 0.02
        #print("ControlTurn Right:", ControlTurn)  
        if (round(targetRotation,2) == round(ControlTurn,2)):
          time.sleep(1)
        forward()
     elif detected[0] == True and detected[2] != library:
          #("if non target object detected")
          backward()
          time.sleep(1)
          detcoord = detected[1]
          detangle = math.atan2(detcoord[1], detcoord[0])
          devAngle = (angle - detangle) 
          devRot   = computeRotation(devAngle)#compute the deviation angle
          v_dev = 1.0
          o_dev = 1.0 * devRot
          vr_dev = v_dev +  Dwheel/2 * o_dev
          vl_dev = v_dev -  Dwheel/2 * o_dev
          #print("Velocity Deviation Left:",  vl_dev)
          #print("Velocity Deviation Right:", vr_dev)
          dv_omegaLeft = vl_dev/radius
          dv_omegaLeft = round(dv_omegaLeft, 2)
          dv_omegaRight= vr_dev/radius
          dv_omegaRight= round(dv_omegaRight,2)
          #print("Omega Deviation Left:", dv_omegaLeft)
          #print("Omega Deviation Right:",dv_omegaRight)
          if (round(devRot,2) > round(ControlTurn,2)):
             turnLeft(dv_omegaLeft,  0 )
             ControlTurn = round(ControlTurn,2) + 0.02
             if (round(devRot,2) == round(ControlTurn,2)):
                time.sleep(1)
             backward()
             continue
          elif (round(devRot,2) < round(ControlTurn,2)):
             turnRight(0,   dv_omegaRight )
             ControlTurn = round(ControlTurn,2) + 0.02
             if (round(devRot,2) == round(ControlTurn,2)):
                time.sleep(1)
             backward()
             continue
     if round(distance) <=1.23:# and detected[2] == library:
        '''print("Pepper: we are at the  Library")
        sayInfront = gTTS(text = "we are at the  Library", lang='en', slow=False) 
        sayInfront.save("front.mp3")
        playsound("front.mp3")'''
        flag = False
        stop()
        #break#[flag , distance]
        #break
        





def GoToClassroom():
  #global flag
  flag = True 
  global classroom
  ControlTurn = 0
  while flag:
     detected  = ObjectDetect()
     robdata = getRobotPos() 
     classroomData = classroomPos()
     #print("location:", classroom)
     #print("detected results:", detected) 
     #print("robot position  data:", robdata) 
     #print("office position data:", classroomData)
     distangle = distance_and_angle(robdata,  classroomData, inDegrees = False)
     radius = computeRadius()
     distance = distangle[0]
     angle = distangle[1]
     #print("distance:", distance)
     #print("distance and angle:", distangle)
     #print("distance wheel:", Dwheel)  
     #print("radius:", radius)
     targetRotation = computeRotation(angle)
     #print("new angle:", targetRotation)
     #V_linear, O_angular, VR => Speed Right, VL => Speed Right 
     V_rob = 0.1
     O_rob = 0.1 * targetRotation
     VR = V_rob + Dwheel/2 * O_rob
     VL = V_rob - Dwheel/2 * O_rob
     #print("Velocity Left:", VL)
     #print("Velocity Right:", VR)
     #compute the omega ...
     Omega_Left = VL/radius
     Omega_Left = round(Omega_Left,2)
     Omega_Right = VR/radius
     Omega_Right = round(Omega_Right,2) 
     #print("Omega Left:", Omega_Left)
     #print("Omega Right:", Omega_Right)

     #drive Robot...
     
     if (round(targetRotation,2) > round(ControlTurn,2)):
        turnLeft(Omega_Left,  0 )
        ControlTurn = round(ControlTurn,2) + 0.02
        #print("ControlTurn Left:", ControlTurn)
       
        if (round(targetRotation,2) == round(ControlTurn,2)):
           time.sleep(1)
        forward()
     elif  (round(targetRotation,2) < round(ControlTurn,2)):
        turnRight(0,  Omega_Right)
        ControlTurn = round(ControlTurn,2) - 0.02
        #print("ControlTurn Right:", ControlTurn) 
        if (round(targetRotation,2) == round(ControlTurn,2)):
          time.sleep(1)
        forward()
     elif detected[0] == True and detected[2] != classroom:
          #print("if non target object detected")
          backward()
          time.sleep(1)
          detcoord = detected[1]
          detangle = math.atan2(detcoord[1], detcoord[0])
          devAngle = (angle - detangle) #+ 10
          devRot   = computeRotation(devAngle)#compute the deviation angle
          v_dev = 1.0
          o_dev = 1.0 * devRot
          vr_dev = v_dev +  Dwheel/2 * o_dev
          vl_dev = v_dev -  Dwheel/2 * o_dev
          #print("Velocity Deviation Left:",  vl_dev)
          #print("Velocity Deviation Right:", vr_dev)
          dv_omegaLeft = vl_dev/radius
          dv_omegaLeft = round(dv_omegaLeft, 2)
          dv_omegaRight= vr_dev/radius
          dv_omegaRight= round(dv_omegaRight,2)
          #print("Omega Deviation Left:", dv_omegaLeft)
          #print("Omega Deviation Right:",dv_omegaRight)
          if (round(devRot,2) > round(ControlTurn,2)):
             turnLeft(dv_omegaLeft,  0 )
             ControlTurn = round(ControlTurn,2) + 0.02
             if (round(devRot,2) == round(ControlTurn,2)):
                time.sleep(1)
             backward()
             continue
          elif (round(devRot,2) < round(ControlTurn,2)):
             turnRight(0,   dv_omegaRight )
             ControlTurn = round(ControlTurn,2) + 0.02
             if (round(devRot,2) == round(ControlTurn,2)):
                time.sleep(1)
             backward()
             continue
     if round(distance) <=1.23:# and detected[2] == classroom:
        '''print("Pepper: we are at the  Classroom")
        sayInfront = gTTS(text = "we are at the  Classroom", lang='en', slow=False) 
        sayInfront.save("front.mp3")
        playsound("front.mp3")'''
        flag = False
        stop()
        #break#[flag , distance]
        #break
        
        #return[distance, classroom]
  






def GoToCafeteria():
  #global flag
  flag = True 
  global cafeteria
  ControlTurn = 0
  while flag:
     detected  = ObjectDetect()
     robdata = getRobotPos() 
     cafeteriaData = getCafeteriaPos()
     #print("location:", cafeteria)
     #print("detected results:", detected) 
     #print("robot position  data:", robdata) 
     #print("office position data:",  cafeteriaData)
     distangle = distance_and_angle(robdata,   cafeteriaData, inDegrees = False)
     radius = computeRadius()
     distance = distangle[0]
     angle = distangle[1]
     #print("distance:", distance)
     #print("distance and angle:", distangle)
     #print("distance wheel:", Dwheel)  
     #print("radius:", radius)
     targetRotation = computeRotation(angle)
     #print("new angle:", targetRotation)
     #V_linear, O_angular, VR => Speed Right, VL => Speed Right 
     V_rob = 0.1
     O_rob = 0.1 * targetRotation
     VR = V_rob + Dwheel/2 * O_rob
     VL = V_rob - Dwheel/2 * O_rob
     #print("Velocity Left:", VL)
     #print("Velocity Right:", VR)
     #compute the omega ...
     Omega_Left = VL/radius
     Omega_Left = round(Omega_Left,2)
     Omega_Right = VR/radius
     Omega_Right = round(Omega_Right,2) 
     #print("Omega Left:", Omega_Left)
     #print("Omega Right:", Omega_Right)

     #drive Robot...
     
     if (round(targetRotation,2) > round(ControlTurn,2)):
        turnLeft(Omega_Left,  0 )
        ControlTurn = round(ControlTurn,2) + 0.02
        #print("ControlTurn Left:", ControlTurn)
       
        if (round(targetRotation,2) == round(ControlTurn,2)):
           time.sleep(1)
        forward()
     elif  (round(targetRotation,2) < round(ControlTurn,2)):
        turnRight(0,  Omega_Right)
        ControlTurn = round(ControlTurn,2) - 0.02
        #print("ControlTurn Right:", ControlTurn)   
        if (round(targetRotation,2) == round(ControlTurn,2)):
          time.sleep(1)
        forward()
     elif detected[0] == True and detected[2] != cafeteria:
          #print("wrong location detected")
          backward()
          time.sleep(1)
          detcoord = detected[1]
          detangle = math.atan2(detcoord[1], detcoord[0])
          devAngle = (angle - detangle)# + 10
          devRot   = computeRotation(devAngle)#compute the deviation angle
          v_dev = 1.0
          o_dev = 1.0 * devRot
          vr_dev = v_dev +  Dwheel/2 * o_dev
          vl_dev = v_dev -  Dwheel/2 * o_dev
          #print("Velocity Deviation Left:",  vl_dev)
          #print("Velocity Deviation Right:", vr_dev)
          dv_omegaLeft = vl_dev/radius
          dv_omegaLeft = round(dv_omegaLeft, 2)
          dv_omegaRight= vr_dev/radius
          dv_omegaRight= round(dv_omegaRight,2)
          #print("Omega Deviation Left:", dv_omegaLeft)
          #print("Omega Deviation Right:",dv_omegaRight)
          if (round(devRot,2) > round(ControlTurn,2)):
             turnLeft(dv_omegaLeft,  0 )
             ControlTurn = round(ControlTurn,2) + 0.02
             if (round(devRot,2) == round(ControlTurn,2)):
                time.sleep(1)
             backward()
             continue
          elif (round(devRot,2) < round(ControlTurn,2)):
             turnRight(0,   dv_omegaRight )
             ControlTurn = round(ControlTurn,2) + 0.02
             if (round(devRot,2) == round(ControlTurn,2)):
                time.sleep(1)
             backward()
             continue
          
     if round(distance,2) <=1.23: # and detected[2] == cafeteria:
        '''print("Pepper: we are at the  Cafeteria")
        sayInfront = gTTS(text = "we are at the  Cafeteria", lang='en', slow=False) 
        sayInfront.save("front.mp3")
        playsound("front.mp3")'''
        flag = False
        stop()
        #break#[flag , distance]
        #break



def GoToDefaultPosition():
  #global flag
  flag = True 
  global defPosition
  ControlTurn = 0
  while flag:
     detected  = ObjectDetect()
     robdata = getRobotPos() 
     defPosData = getDefaultPosition()
     #print("location:", defPosition)
     #print("detected results:", detected) 
     #print("robot position  data:", robdata) 
     #print("default position data:",  defPosData)
     distangle = distance_and_angle(robdata,   defPosData, inDegrees = False)
     radius = computeRadius()
     distance = distangle[0]
     angle = distangle[1]
     #print("distance:", distance)
     #print("distance and angle:", distangle)
     #print("distance wheel:", Dwheel)  
     #print("radius:", radius)
     targetRotation = computeRotation(angle)
     #print("new angle:", targetRotation)
     #V_linear, O_angular, VR => Speed Right, VL => Speed Right 
     V_rob = 0.1
     O_rob = 0.1 * targetRotation
     VR = V_rob + Dwheel/2 * O_rob
     VL = V_rob - Dwheel/2 * O_rob
     #print("Velocity Left:", VL)
     #print("Velocity Right:", VR)
     #compute the omega ...
     Omega_Left = VL/radius
     Omega_Left = round(Omega_Left,2)
     Omega_Right = VR/radius
     Omega_Right = round(Omega_Right,2) 
     #print("Omega Left:", Omega_Left)
     #print("Omega Right:", Omega_Right)

     #drive Robot...
     
     if (round(targetRotation,2) > round(ControlTurn,2)):
        turnLeft(Omega_Left,  0 )
        ControlTurn = round(ControlTurn,2) + 0.02
        #print("ControlTurn Left:", ControlTurn)
       
        if (round(targetRotation,2) == round(ControlTurn,2)):
           time.sleep(1)
        forward()
     elif  (round(targetRotation,2) < round(ControlTurn,2)):
        turnRight(0,  Omega_Right)
        ControlTurn = round(ControlTurn,2) - 0.02
        print("ControlTurn Right:", ControlTurn)  
        if (round(targetRotation,2) == round(ControlTurn,2)):
          time.sleep(1)
        forward()
     elif detected[0] == True and detected[2] != defaultPos:
          print("wrong location detected")
          backward()
          time.sleep(1)
          detcoord = detected[1]
          detangle = math.atan2(detcoord[1], detcoord[0])
          devAngle = (angle - detangle)# + 10
          devRot   = computeRotation(devAngle)#compute the deviation angle
          v_dev = 1.0
          o_dev = 1.0 * devRot
          vr_dev = v_dev +  Dwheel/2 * o_dev
          vl_dev = v_dev -  Dwheel/2 * o_dev
          #print("Velocity Deviation Left:",  vl_dev)
          #print("Velocity Deviation Right:", vr_dev)
          dv_omegaLeft = vl_dev/radius
          dv_omegaLeft = round(dv_omegaLeft, 2)
          dv_omegaRight= vr_dev/radius
          dv_omegaRight= round(dv_omegaRight,2)
          #print("Omega Deviation Left:", dv_omegaLeft)
          #print("Omega Deviation Right:",dv_omegaRight)
          if (round(devRot,2) > round(ControlTurn,2)):
             turnLeft(dv_omegaLeft,  0 )
             ControlTurn = round(ControlTurn,2) + 0.02
             if (round(devRot,2) == round(ControlTurn,2)):
                time.sleep(1)
             backward()
             continue
          elif (round(devRot,2) < round(ControlTurn,2)):
             turnRight(0,   dv_omegaRight )
             ControlTurn = round(ControlTurn,2) + 0.02
             if (round(devRot,2) == round(ControlTurn,2)):
                time.sleep(1)
             backward()
             continue
          
     if round(distance,2) <=1.23: # and detected[2] == default position:
        '''print("Pepper: I'm in the waiting Area")
        sayInfront = gTTS(text = "I'm in the waiting Area", lang='en', slow=False) 
        sayInfront.save("front.mp3")
        playsound("front.mp3")'''
        flag = False
        stop()
        #break#[flag , distance]
        #break

        
'''
def menu():
    print("Select 1, to go to the Office")
    print("Select 2, to go to the library")
    print("Select 3, to go to the Classroom")
    print("Select 4, to go to the Cafeteria")
    print("Select 5, to go to the Default Position")
    print("Select 0, to go to the exit")
    #print("Select 2, to go the library"
def main():
    x = ''
    while x!=0:
       menu()
       x = int(input("location selection:"))
       if x == 1:#office
          GoToOffice()
          
       elif x == 2:#library
          GoToLibrary()
          
       elif x == 3:#classroom
         GoToClassroom()
         
       elif x == 4:#cafeteria
          GoToCafeteria()
       elif x == 5: #Default Position
          GoToDefaultPosition()
       else:
          continue
       
         
             


if __name__ == "__main__":
   main()
'''



#import the necessary libraries...
import io
import os
import re
import qi
import csv
import cv2
import time
import naoqi
import motion
import random
import pyttsx3
import numpy as np
import simplejson as json
import speech_recognition as sr

from PIL import Image
from naoqi import ALProxy
from myConnection import IP, PORT, RobotConnection

def DetectFaces(motionService):
    HeadJoint = ["HeadPitch", "HeadYaw"]
    HeadAngles = [-5,6, 0]
    HeadAngles= [x * motion.TO_RAD for x in HeadAngles]
    motionService.angleInterpolationWithSpeed(HeadJoint, HeadAngles, 0.6)


    cascadePath = "haarcascade_frontalface_default.xml"
    faceCascade = cv2.CascadeClassifier(cascadePath)
    vidobj = cv2.VideoCapture(0)
    while True:
        ret, frame = vidobj.read()
        frame = cv2.resize(frame, (320,240))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = faceCascade.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(25, 25),
        flags=cv2.CASCADE_SCALE_IMAGE
        )
    
        lenFaces = len(faces)
        #print("length of faces:", lenFaces)
        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.imshow('Detecting faces', frame)
    
        if cv2.waitKey(1) & 0xFF == ord('q') or lenFaces != 0:
           break


    vidobj.release()
    cv2.destroyAllWindows()
    return faces, lenFaces   



#!/usr/bin/env python
import io
import os
import re
import qi
import csv
import time
import vrep
import rospy
import naoqi
import random
import pyttsx3
import numpy as np
import simplejson as json
import speech_recognition as sr

from gtts import gTTS 
from naoqi import ALProxy
from playsound import playsound
from pepperVision import DetectFaces
from nltk.tokenize import word_tokenize
from myConnection import IP, PORT, RobotConnection

from pepper_movements  import direction_left, go_forward,  direction_right,  Hello_Gesture, Make_Gesture
from driverobot import *

#initialize the rospy node...
rospy.init_node("pepperSpeech_Movement", anonymous=True)
rospy.loginfo("\nTitle:Start Robot Path Planning Program.\n Author(s): Ugochukwu Alex Gbenimachor, Suleyman Balashzade \n  Date: TBD")

'''
author(s):
Date: 
Aim of the Application: To move pepper with the Speech data.json and we are interleaving the motion with speech...
'''

#initialize robot session... 
PORT  = PORT
RoboIP=IP
session = RobotConnection(RoboIP, PORT)
#define motiion service...

motion_service = session.service("ALMotion")
motion_service.wakeUp()
tts_service = session.service("ALTextToSpeech")


#initialize the Python Speech Text
engine = pyttsx3.init() # object creation
#lets load the json for the speech data and clean...
filename ='intentCsv.json'
def readjson(filen):
    loadfile = open(filen,'r')
    datafile = json.load(loadfile)
    return datafile
    
datafile = readjson(filename)
print(datafile[0]['text'])    
#define function initiate communication...
def SpeakToMe():
    
    r = sr.Recognizer()
    mic= sr.Microphone()
    with mic as source:
         print("Say something")
         r.adjust_for_ambient_noise(source)
         audio = r.listen(source)
         
    speechTalk=''     
    
    try:
       speechTalk =  r.recognize_google(audio)
       print("USER: ", speechTalk)
    except sr.UnknownValueError:
       print("Please, I did not get that")
    except sr.RequestError:   
        print("Error in your network Connection...")
        
    return speechTalk
       
#''' Greetings '''       
#function for greeting people...
def SayGreetings():
    response =''
    responsez=''
    #myname = ''
    #lets say hello
    textz = SpeakToMe()
    textz = textz.encode('utf-8')
    texts = textz.split()
    inputs= [n.lower() for n in datafile[0]['text']]
    #outputs = [o for o in datafile[0]['responses']]
    
    print(inputs)
    for x  in  texts:
        if x.lower() in inputs:
          
           responses = random.choice(datafile[0]['responses'])
           responsez = [ "friend" if x == "<Human>" else x  for x in responses.split()]
           
        else:
            print("Please I didn't get that")
    res = " ".join(responsez)
    return res


#'''Extraction of wordlist'''
#function for building word list...
def buildWordlist(textDatalist, keywordlist):
    accompany_wordlist = list()
    for text in textDatalist:
        text = text.split()
        for x in text:
            if x not in accompany_wordlist:
               accompany_wordlist.append(x)
        acco_wordlist = [n.lower() for n in  accompany_wordlist if n != "?"]
        for x in keywordlist:
            if x not in acco_wordlist:
               acco_wordlist.append(x)
        acco_wordlist = [n.lower() for n in acco_wordlist if n not in ["<robot>", "office", "library", "cafeteria", "classroom", "hi", "how", "to", "hello","can", "you","please","on","the","me"]]
    return acco_wordlist


#########____'''Direction  Part'''____#########
#this part will ensure that the robot is able to process the It's speech...
AccompanyKeywords = ["lead","accompany", "assist", "take", "follow","bye", "thank you"]#define the Accompany keywords
InformationKeywords=["get", "tell", "show", "give", "information", "direct","direction","bye", "thank you"]#define the Information keywords


#build keyword list for the robot..    
accoDatalist = datafile[1]['text'] #building a dictionary of keywords from the text json file 
accompany_wordlist =   buildWordlist(accoDatalist, AccompanyKeywords) #accompany keyword list...
print(accompany_wordlist) 


#build information keyword list...
information_Datalist = datafile[6]['text']
information_keywordlist = buildWordlist(information_Datalist, InformationKeywords)
print(information_keywordlist)


def TellDirection(sentence):
    print(sentence)
    sentence = sentence.encode("utf-8")
    sentence = sentence.split()
    sentence = [n.lower() for n in sentence]
    for word in sentence:
        if word.lower() in accompany_wordlist and "office" in sentence:
           #return office response if accompany...
           TaccompanyResponse = datafile[1]['responses']
           TaccompanyResponse = " ".join(TaccompanyResponse)
           officeAccResponse = [n.lower() for n in  TaccompanyResponse.split()]
           print("Accompany office response:", officeAccResponse)
           accSpeak = gTTS(text = "please follow me to the office", lang='en', slow=False)
           accSpeak.save("speech.mp3")
           playsound("speech.mp3")
           GoToOffice()
                   
               
                 
               
                       
         
        elif word.lower() in  information_keywordlist and "office" in sentence: 
             #return toilet response if information
            
             toiletResponseinfo = datafile[5]['responses']
             toiletResponseinfo = " ".join(toiletResponseinfo)
             toiletResponseinfo = [n.lower() for n in  toiletResponseinfo.split()]
             print("Information office response:", toiletResponseinfo)
             firstSent = toiletResponseinfo[0]
             lastSent  = toiletResponseinfo[3:]
             for speech in toiletResponseinfo:
               if speech == firstSent:
                  print("Pepper:", "to go to the office")
                  speakFirst = gTTS(text = "to go to the office", lang='en', slow=False)
                  speakFirst.save("speech.mp3")
                  playsound("speech.mp3")
                  
               elif speech == "<turn_left>":
                    print("Pepper:", "you will turn left")
                    sayLeft = gTTS(text = "you will turn left", lang='en', slow=False)
                    
                    sayLeft.save("left.mp3")
                    playsound("left.mp3")
                    tts_service.say("you will turn left")
                    direction_left(motion_service)sayLeft = gsayLeft = gTTS(text = "you will turn left", lang='en', slow=False)
                    
                    sayLeft.save("left.mp3")
                    playsound("left.mp3")TTS(text = "you will turn left", lang='en', slow=False)
                    
                    sayLeft.save("left.mp3")
                    playsound("left.mp3")
                    #break
               elif speech == "<go_forward>":
                    print("Pepper:", "you will walk straight ")
                    sayForward = gTTS(text = "you will walk straight  ", lang='en', slow=False)
                    sayForward.save("straight.mp3")
                    playsound("straight.mp3")
                    tts_service.say("you will walk straight  ")
                    go_forward(motion_service)
                    #break
               elif speech == "<turn_right>":
                    print("Pepper:", "you will turn to your right ")
                    sayRight = gTTS(text = "you will turn to your right", lang='en', slow=False)
                    sayRight.save("right.mp3")
                    playsound("right.mp3")
                    tts_service.say("you will turn to your right")
                    direction_right(motion_service)
                    #break 
               elif speech == "<in_front>":
                    print("Pepper:", "you'll see the office")
                    sayInfront = gTTS(text = "you'll see the office", lang='en', slow=False) 
                    sayInfront.save("front.mp3")
                    playsound("front.mp3")
                    tts_service.say("you'll see the office")
                    break
                    
                      
        
        elif word.lower() in accompany_wordlist and "classroom" in sentence:    
            
            #return classroom accompany
            classAccResponse = datafile[3]['responses']
            classAccResponse = " ".join(classAccResponse)
            classroomAccResponse = [n.lower() for n in classAccResponse.split()]
            print("classroom Accompany response:",classroomAccResponse) 
            accSpeak = gTTS(text = "please follow me to the classroom", lang='en', slow=False)
            accSpeak.save("speech.mp3")
            tts_service.say("please follow me to the classroom")
            playsound("speech.mp3")
            GoToClassroom()               
                
             
         
        elif word.lower() in information_keywordlist and "classroom" in sentence:    
            
            #return classroom information
            classInfoResponse =  datafile[7]['responses']
            classInfoResponse = " ".join(classInfoResponse)
            classroomInfoResponse = [n.lower() for n in classInfoResponse.split()]
            print("information classroom response:", classroomInfoResponse)
            firstSent = classroomInfoResponse[0]
            lastSent  = classroomInfoResponse[3:] 
            for speech in classroomInfoResponse:
                if speech == firstSent:
                   print("Pepper:", "to go to the classroom")
                   speakFirst = gTTS(text = "to go to the classroom", lang='en', slow=False)
                   speakFirst.save("speech.mp3")
                   playsound("speech.mp3")
                   time.sleep(1)
                elif speech == "<turn_left>":
                     print("Pepper:", "you will make a left turn")
                     sayLeft = gTTS(text = "you will make a left turn ", lang='en', slow=False)
                     sayLeft.save("left.mp3")
                     playsound("left.mp3")
                     tts_service.say("you will make a left turn ")
                     direction_left(motion_service)
                elif speech == "<go_forward>":
                     print("Pepper:", "then you will, walk straight")
                     sayForward = gTTS(text = "then you will, walk straight", lang='en', slow=False)
                     sayForward.save("straight.mp3")
                     playsound("straight.mp3")
                     tts_service.say("then you will, walk straight")
                     go_forward(motion_service)
                elif speech == "<turn_right>":
                     print("Pepper:", "you will make a right turn")
                     sayRight = gTTS(text = "you will make a right turn", lang='en', slow=False)
                     sayRight.save("right.mp3")
                     playsound("right.mp3")
                     tts_service.say("you will make a right turn")
                     direction_right(motion_service)
                elif speech == "<in_front>":
                     print("Pepper:", "you will see the classroom")
                     sayInfront = gTTS(text = "you will see the classroom", lang='en', slow=False) 
                     sayInfront.save("front.mp3")
                     playsound("front.mp3")
                     tts_service.say("you will see the classroom")
                     break
                     
                 

        elif word.lower() in accompany_wordlist and "library" in sentence:    
            
            #return library accompany 
            libraryAccResponse = datafile[2]['responses']
            libraryAccResponse = " ".join(libraryAccResponse)
            libraryaccompany = [n.lower() for n in libraryAccResponse.split()]
            print("library accompany response:", libraryaccompany)
            print("Pepper:", "please follow me to the library")
            accSpeak = gTTS(text = "please follow me to the library", lang='en', slow=False)
            accSpeak.save("speech.mp3")
            playsound("speech.mp3")
            GoToLibrary()   
                
               
             
         
        elif word.lower() in information_keywordlist and "library" in sentence:    
            
            #return library information
            libraryResponseInfo =  datafile[6]['responses']
            libraryResponseInfo = " ".join(libraryResponseInfo)
            libraryInfo = [n.lower() for n in  libraryResponseInfo.split()]
            print("Information library response:",libraryInfo)
            firstSent = libraryInfo[0]
            lastSent  = libraryInfo[3:] 
            for speech in libraryInfo:
                if speech == firstSent:
                   print("Pepper:",  "to go to the library")
                   speakFirst = gTTS(text = "to go to the library", lang='en', slow=False)
                   speakFirst.save("speech.mp3")
                   playsound("speech.mp3")
                   tts_service.say("to go to the library")
                elif speech == "<turn_left>":
                   print("Pepper:",  "you will turn left ")
                   sayLeft = gTTS(text = "you will turn left", lang='en', slow=False)
                   sayLeft.save("left.mp3")
                   playsound("left.mp3")
                   tts_service.say("you will turn left")
                   direction_left(motion_service)
                elif speech == "<go_forward>":
                   print("Pepper:", "you will walk straight")
                   sayForward = gTTS(text = "you will walk straight", lang='en', slow=False)
                   sayForward.save("straight.mp3")
                   playsound("straight.mp3")
                   tts_service.say("you will walk straight")
                   go_forward(motion_service)
                elif speech == "<turn_right>":
                   print("Pepper:", "you will make a right turn")
                   sayRight = gTTS(text = "you will make a right turn", lang='en', slow=False)
                   sayRight.save("right.mp3")
                   playsound("right.mp3")
                   tts_service.say("you will make a  right turn")
                   direction_right(motion_service)
                elif speech == "<in_front>":
                   print("Pepper:", "you will see  the library")
                   sayInfront = gTTS(text = "you will see  the library", lang='en', slow=False) 
                   sayInfront.save("front.mp3")
                   playsound("front.mp3")
                   tts_service.say("you will see  the library")
                   break
                  
                
        elif word.lower() in accompany_wordlist and "cafeteria" in sentence:    
            
            #return cafeteria accompany 
            cafeteriaAccResponse = datafile[4]['responses']
            cafeteriaAccResponse = " ".join(cafeteriaAccResponse)
            cafeteriaccompany = [n.lower() for n in cafeteriaAccResponse.split()]
            print("cafateria accompany response:",cafeteriaccompany)
            print("Pepper:", "please follow me to the cafateria")
            accSpeak = gTTS(text = "please follow me to the cafateria", lang='en', slow=False)
            accSpeak.save("speech.mp3")
            playsound("speech.mp3")
            GoToCafeteria()                   
                               
                
         
        elif word.lower() in information_keywordlist and "cafeteria" in sentence:    
            
            #return cafeteria information
            cafeteriaResponseInfo =  datafile[8]['responses']
            cafeteriaResponseInfo = " ".join(cafeteriaResponseInfo)
            cafeteriaInfo = [n.lower() for n in  cafeteriaResponseInfo.split()]
            print("information cafateria response:",cafeteriaInfo)
            firstSent = cafeteriaInfo[0]
            lastSent  = cafeteriaInfo[3:] 
            for speech in cafeteriaInfo:
                if speech == firstSent:
                   print("Pepper:", "to go to the cafeteria")
                   speakFirst = gTTS(text = "to go to the cafeteria", lang='en', slow=False)
                   speakFirst.save("speech.mp3")
                   playsound("speech.mp3")
                   tts_service.say("to go to the cafeteria")
                   
                elif speech == "<turn_left>":
                   print("Pepper:", "you will make a left turn")
                   sayLeft = gTTS(text = "you will make a left turn", lang='en', slow=False)
                   sayLeft.save("left.mp3")
                   playsound("left.mp3")
                   tts_service.say("you will make a left turn")
                   direction_left(motion_service)
                elif speech == "<go_forward>":
                   print("Pepper:", "you go some few steps forward")
                   sayForward = gTTS(text = "you go some few steps forward", lang='en', slow=False)
                   sayForward.save("straight.mp3")
                   tts_service.say("you go some few steps forward")
                   playsound("straight.mp3")
                   go_forward(motion_service)
                elif speech == "<turn_right>":
                   print("Pepper:", "you will make a right turn ")
                   sayRight = gTTS(text = "you will make a right turn", lang='en', slow=False)
                   sayRight.save("right.mp3")
                   tts_service.say("you will make a right turn")
                   playsound("right.mp3")
                   direction_right(motion_service)
                elif speech == "<in_front>":
                   sayInfront = gTTS(text = "you will see cafeteria", lang='en', slow=False) 
                   sayInfront.save("front.mp3")
                   tts_service.say("you will see cafeteria")
                   playsound("front.mp3")
                   break
                   
               
        
       
                    
                  
if __name__ =="__main__":
   close = ""
   while close != "bye" or close !="thank you" or "no thank you":
      #pepper detects persons face...
      faces, lenFaces = DetectFaces(motion_service)
      if lenFaces != 0:
         #move closer to guest ...
         move_forward(motion_service)
         #say hello to Guest and provide instruction...
         Hello_Gesture(motion_service)
         sayHello = gTTS(text="Hello , I can accompany you to your destination or give you a an information on how to get there. " , lang='en', slow=False)
         
         print("Pepper:", "Hello , I can accompany you to your destination or give you a an information on how to get there.")
         sayHello.save("hello.mp3")
         playsound("hello.mp3")
        
         
         #greetings... 
         mylist = ["hello", "hi"]
         try:
            greet = SayGreetings()
         
         except AssertionError:
             greet = random.choice(mylist)
         #print("user:", greet)
         greetings = gTTS(text = greet, lang='en', slow=False) 
         greetings.save("greeting.mp3")
         playsound("greeting.mp3")
         sentence = SpeakToMe()
         Make_Gesture(motion_service)
         
         #print("User:", sentence)
         TellDirection(sentence)
         #closing... 
         closeDirection = gTTS(text="Is there anything else i can do for you? " , lang='en', slow=False)
         print("Pepper:", "Is there anything else i can do for you?") 
         closeDirection.save("closing.mp3")
         playsound("closing.mp3")
         close = SpeakToMe()
         close = close.encode("utf-8")
         
         
         
         #print("user:":close)
         if close == "yes":
            closeDirection = gTTS(text="What else can i do for you? " , lang='en', slow=False)
            print("Pepper:", "What else can i  do for you?") 
            closeDirection.save("closing.mp3")
            playsound("closing.mp3")
            sentence = SpeakToMe()
            TellDirection(sentence)           
         #closing... 
         
        
         
         elif close == "bye" or close == "thank you" or "no thank you":
            sayNice =  gTTS(text="Okay, I'm glad, I could help. It's my pleasure, bye now." , lang='en', slow=False)
            print("Pepper:", "Okay, I'm glad, I could help. It's my pleasure, bye now.") 
            sayNice.save("saynice.mp3")
            tts_service.say(close)
            playsound("saynice.mp3")
            GoToDefaultPosition()
            break
#'''        
       
       

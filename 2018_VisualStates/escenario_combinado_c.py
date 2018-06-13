#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys, threading, time, signal
sys.path.append("/opt/jderobot/lib/python2.7")
sys.path.append("/opt/jderobot/lib/python2.7/visualStates_py")
from codegen.python.state import State
from codegen.python.temporaltransition import TemporalTransition
from codegen.python.conditionaltransition import ConditionalTransition
from codegen.python.runtimegui import RunTimeGui
from PyQt5.QtWidgets import QApplication
import config, comm
import cv2
import numpy as np
import apriltag
import math
import xml.etree.ElementTree as ET
import time
import datetime
import traceback
import logging


class State0(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		pass

class State1(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		logging.basicConfig(filename='April',format='%(asctime)s %(message)s',level=logging.INFO,filemode='w')
		
		vx=0
		vy=0
		vz=0
		vw=0
		error_xy = [0,0]
		error_z = 0
		error_w = 0
		onTarget=False
		
		#Image Processing
		inputImage = self.interfaces.myCamera.getImage()
		inputImageData = inputImage.data
		inputImageData_h = inputImage.height
		inputImageData_w = inputImage.width
		inputImageData.shape = inputImageData_h, inputImageData_w, 3
		color = cv2.cvtColor(inputImageData, cv2.COLOR_RGB2BGR)
		#Apriltags Detection
		gray = cv2.cvtColor(inputImageData, cv2.COLOR_RGB2GRAY)
		#cv2.imshow("Image",gray)
		#cv2.waitKey(30)
		detections = self.interfaces.detector.detect(gray)
		num_detections = len(detections)
		if (num_detections <= 0):
		    self.interfaces.x = 0.0
		    self.interfaces.y = 0.0
		    self.interfaces.z = 0.0
		    self.interfaces.pitch = 0.0
		else:
		    for i,detection in enumerate(detections):
		        if (detection.tag_id == self.interfaces.beacons[self.interfaces.beaconCounter]):
		            onTarget=True
		            print "Beacon %d detected" % detection.tag_id
		            '''
		            HALF_MARKER_SIZE=self.interfaces.MARKER_SIZE/2 #Size in meters
		            m_MarkerPoints = np.zeros((4,3),dtype=np.float32)
		            m_MarkerPoints[0,0] =-HALF_MARKER_SIZE
		            m_MarkerPoints[0,1] =-HALF_MARKER_SIZE
		            m_MarkerPoints[0,2] = 0
		            m_MarkerPoints[1,0] =-HALF_MARKER_SIZE
		            m_MarkerPoints[1,1] = HALF_MARKER_SIZE
		            m_MarkerPoints[1,2] = 0
		            m_MarkerPoints[2,0] = HALF_MARKER_SIZE
		            m_MarkerPoints[2,1] = HALF_MARKER_SIZE
		            m_MarkerPoints[2,2] = 0
		            m_MarkerPoints[3,0] = HALF_MARKER_SIZE
		            m_MarkerPoints[3,1] = -HALF_MARKER_SIZE
		            m_MarkerPoints[3,2] = 0
		            #Camera Matrix and Distorssion Coefficients. 
		            #Use cameracalib from JdeRobot for calibrating a different camera than ArDrone 2 one
		            #Camera Matrix 
		                #Gazebo simulator [187.336, 0, 160, 0, 187.336, 120, 0, 0, 1]
		                #ArDrone 2        [731.37257937739332, 0, 322.35647387552422, 0, 734.23757205576192, 246.30430666269825, 0, 0, 1]
		            cameraMatrix = np.array([731.37257937739332, 0, 322.35647387552422, 0, 734.23757205576192, 246.30430666269825, 0, 0, 1])
		            cameraMatrix = np.resize(cameraMatrix,(3,3))        
		            #Distorsion Coefficients 
		                #Gazebo         np.zeros((5, 1), np.uint8) #Comment this line when using ArDrone 2
		                #Ardrone 2      np.array([-0.07304604105914128, 0.50646907582979650, 0.00024443957708413687, 0.00074556540195940392,-1.0762308065763191])
		            distCoeffs = np.array([-0.07304604105914128, 0.50646907582979650, 0.00024443957708413687, 0.00074556540195940392,-1.0762308065763191]) #Substitute Coefficients
		            '''
		            retVal,rvec,tvec =cv2.solvePnP(self.interfaces.m_MarkerPoints
		                                        ,detection.corners
		                                        ,self.interfaces.cameraMatrix
		                                        ,self.interfaces.distCoeffs)
		            rodri = cv2.Rodrigues(rvec)
		            #We get X,Y and Z
		            cameraPosition = -np.matrix(rodri[0]).T * np.matrix(tvec) 
		            xApril = detection.center[0]
		            yApril = detection.center[1]
		            self.interfaces.x = cameraPosition.item(0) #Center needs to be changed
		            self.interfaces.y = cameraPosition.item(1) #Center needs to be changed
		            self.interfaces.z = cameraPosition.item(2)
		            #We get roll, pitch and yaw from Euler Angles        
		            eulerAngles = self.interfaces.rotationMatrixToEulerAngles(rodri[0])
		            #yaw = eulerAngles[0]
		            #pitch = eulerAngles[1]
		            #roll = self.interfaces.rollCorrection(eulerAngles[2])
		            #Threshold for pitch correction (center of the width of the image)
		            th=inputImageData_w/2
		            self.interfaces.pitch = self.interfaces.pitchCorrection(xApril,th,eulerAngles.item(1))
		            print xApril, yApril, self.interfaces.pitch
		        
		            logging.info ("1-x %.3f;" % self.interfaces.x + "y %.3f;" % self.interfaces.y + "z %.3f;" % self.interfaces.z + "pitch %.3f" % self.interfaces.pitch)
		            if (abs(self.interfaces.x)>0 or abs(self.interfaces.y)>0 or abs(self.interfaces.z)>0 or abs(self.interfaces.pitch)>0):
		                #PID controller
		                error_xy = [self.interfaces.center[0]-self.interfaces.x, self.interfaces.center[1]-self.interfaces.y]
		                error_z = self.interfaces.min_z - self.interfaces.z
		                error_w = -self.interfaces.pitch
		                #print ("error_x %.4f" % error_xy[0],"error_y %.4f" % error_xy[1], "error_z %.4f" % error_z,"error_pitch %.4f" % error_w)
		                #VX
		                print "vx"
		                if(abs(error_xy[0])<self.interfaces.dead_band_x):
		                    #print "Vy en banda de seguridad"
		                    vx=0
		                    #self.yDeadZone=True
		                else:
		                    #Get vy Speed thanks to the PID controller and checks if in Dead band.
		                    vx,vxiMod = self.interfaces.getPIDSpeed(error_xy[0]
		                                                    ,self.interfaces.error_xy_anterior[0]
		                                                    ,self.interfaces.vxi
		                                                    ,self.interfaces.cycle
		                                                    ,self.interfaces.kp
		                                                    ,self.interfaces.kd
		                                                    ,self.interfaces.ki)
		                    self.interfaces.vxi=vxiMod
		                #vy
		                print "vy"
		                if(abs(error_xy[1])<self.interfaces.dead_band_y):
		                    #print "Vy en banda de seguridad"
		                    vy=0
		                    #self.yDeadZone=True
		                else:
		                    #Get vy Speed thanks to the PID controller and checks if in Dead band.
		                    vy,vyiMod = self.interfaces.getPIDSpeed(error_xy[1]
		                                                    ,self.interfaces.error_xy_anterior[1]
		                                                    ,self.interfaces.vyi
		                                                    ,self.interfaces.cycle
		                                                    ,self.interfaces.kp
		                                                    ,self.interfaces.kd
		                                                    ,self.interfaces.ki)
		                    self.interfaces.vyi=vyiMod
		                #vz
		                print "vz"
		                if(abs(error_z)<self.interfaces.dead_band_z):
		                    #print "vz en banda de seguridad"
		                    vz=0
		                    #self.yDeadZone=True
		                else:
		                    #Get vz Speed thanks to the PID controller and checks if in Dead band.
		                    vz,vziMod = self.interfaces.getPIDSpeed(error_z
		                                                    ,self.interfaces.error_z_anterior
		                                                    ,self.interfaces.vzi
		                                                    ,self.interfaces.cycle
		                                                    ,self.interfaces.kpz
		                                                    ,self.interfaces.kdz
		                                                    ,self.interfaces.kiz)
		                    self.interfaces.vzi=vziMod
		                #vw
		                print "vw"
		                if(abs(error_z)<self.interfaces.dead_band_w):
		                    #print "vw en banda de seguridad"
		                    vw=0
		                    #self.yDeadZone=True
		                else:
		                    #Get vw Speed thanks to the PID controller and checks if in Dead band.
		                    vw,vwiMod = self.interfaces.getPIDSpeed(error_w
		                                                    ,self.interfaces.error_w_anterior
		                                                    ,self.interfaces.vwi
		                                                    ,self.interfaces.cycle
		                                                    ,self.interfaces.kpw
		                                                    ,self.interfaces.kdw
		                                                    ,self.interfaces.kiw)
		                    self.interfaces.vwi=vwiMod   
		                    
		        
		vy=self.interfaces.limitSpeed(vy)
		vx=self.interfaces.limitSpeed(vx)
		vz=self.interfaces.limitSpeed(vz)
		vw=self.interfaces.limitSpeed(vw)
		#Print Speeds
		logging.info ("2-vz:%.3f;" % -vz + "vx:%.3f;" % -vx+  "vy:%.3f;" % vy +"vw:%.3f" % vw) #Pose
		#logging.info ("2-vz:%.3f;" % -vz + "vy:%.3f;" % vy + "vx:%.3f;" % vx +"vw:%.3f" % vw) #AprilTags
		print ("2-vz:%.3f;" % -vz + "vx:%.3f;" % -vx+  "vy:%.3f;" % vy +"vw:%.3f" % vw)        #Pose
		#print ("2-vz:%.3f;" % -vz + "-vy:%.3f;" % vy + "-vx:%.3f;" % vx +"vw:%.3f" % vw)      #AprilTags
		#Send Speeds
		self.interfaces.myCmdvel.sendCMDVel(-vz,-vx,vy,0,0,vw) #Center from pose
		#self.interfaces.myCmdvel.sendCMDVel(-vz,vy,vx,0,0,vw)  #Center from Apriltags
		self.interfaces.error_xy_anterior = error_xy
		self.interfaces.error_z_anterior = error_z
		self.interfaces.error_w_anterior = error_w
		if (onTarget):
		    diff=time.time()-self.interfaces.timeOnTarget
		    if (self.interfaces.timeOnTarget==0):
		        self.interfaces.timeOnTarget=time.time()
		    elif (diff<self.interfaces.timeLimitToChange):
		        print "On target for: "+str(diff)+" seconds"
		    elif (diff>=self.interfaces.timeLimitToChange):
		       print "Changing State"
		       self.interfaces.beaconCounter += 1
		       self.interfaces.switchBeacon = True 
		else:
		    self.interfaces.timeOnTarget=0

class State3(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		if self.interfaces.beaconCounter >= (len(self.interfaces.beacons)):
		    #Landing Time
		    self.interfaces.timeToColor = True
		else:
		    self.interfaces.myCmdvel.sendCMDVel(0,0,0,0,0,-0.2)
		    inputImage = self.interfaces.myCamera.getImage()
		    inputImageData = inputImage.data
		    inputImageData_h = inputImage.height
		    inputImageData_w = inputImage.width
		    inputImageData.shape = inputImageData_h, inputImageData_w, 3
		    color = cv2.cvtColor(inputImageData, cv2.COLOR_RGB2BGR)
		#Apriltags Detection
		    gray = cv2.cvtColor(inputImageData, cv2.COLOR_RGB2GRAY)
		    detections = self.interfaces.detector.detect(gray)
		    num_detections = len(detections)
		    for i, detection in enumerate(detections):
		        if (detection.tag_id == self.interfaces.beacons[self.interfaces.beaconCounter]):
		            self.interfaces.myCmdvel.sendCMDVel(0,0,0,0,0,0)
		            self.interfaces.switchBeacon=True

class State4(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		if(self.interfaces.timeToLand):
			self.interfaces.myExtra.land()
			self.interfaces.timeToLand=False
			print "Landed - End of exercise"
		else:
			pass

class State12(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		logging.basicConfig(filename='color1',format='%(asctime)s %(message)s',level=logging.INFO,filemode='w')
		start_time = datetime.datetime.now()
		
		vx=0
		vy=0
		xError=0
		yError=0
		foundCenter=False
		#Image Processing
		input_image = self.interfaces.myCamera.getImage()
		input_imageData = input_image.data
		centerImage = input_imageData
		hsv = cv2.cvtColor(input_imageData, cv2.COLOR_BGR2HSV)
		maskPrimary,maskRGBPrimary = self.interfaces.hsvFilter(self.interfaces.PH_max,self.interfaces.PS_max,self.interfaces.PV_max,self.interfaces.PH_min,self.interfaces.PS_min,self.interfaces.PV_min,input_image.data,hsv)
		
		maskSecondary,maskRGBSecondary = self.interfaces.hsvFilter(self.interfaces.SH_max,self.interfaces.SS_max,self.interfaces.SV_max,self.interfaces.SH_min,self.interfaces.SS_min,self.interfaces.SV_min,input_image.data,hsv)
		
		kernel = np.ones((3,3),np.uint8)
		
		maskRGBPrimary = cv2.erode(maskRGBPrimary,kernel,iterations = self.interfaces.PErode)
		maskRGBPrimary = cv2.dilate(maskRGBPrimary,kernel,iterations = self.interfaces.PDilate)
		momentsPrimary = cv2.moments(maskPrimary)
		areaPrimary = momentsPrimary['m00']
		
		maskRGBSecondary = cv2.erode(maskRGBSecondary,kernel,iterations = self.interfaces.SErode)
		maskRGBSecondary = cv2.dilate(maskRGBSecondary,kernel,iterations = self.interfaces.SDilate)
		momentsSecondary = cv2.moments(maskSecondary)
		areaSecondary = momentsSecondary['m00']
		
		maskRGBTot = maskRGBPrimary+maskRGBSecondary
		
		#Controller based on how many tiles have been detected
		if(areaPrimary>0 and areaSecondary>0):
		    print "Both colors detected"
		    crosshair,f = self.interfaces.findCrosshair(input_imageData,maskRGBPrimary,maskRGBSecondary)
		    print "F: %d" % len(f)
		    #f represents the number of cuadrants detected
		    if(len(f)>0):
		        #Returns Center of crossair and checks if the center of the crossair was found.        
		        crossY,crossX,foundCenter,centerImage = self.interfaces.getCrosshairCoord(crosshair,centerImage)
		        logging.info("y:%.2f;"% crossY + "x:%.2f" % crossX)
		        if(foundCenter):
		            print "Center Found!"
		            yError=self.interfaces.center[1]-crossY
		            xError=self.interfaces.center[0]-crossX 
		            #Y Controller
		            if(abs(yError)<self.interfaces.yDistanceDeadBand):
		                #print "Vy en banda de seguridad"
		                vy=0
		                #self.yDeadZone=True
		            else:
		                print "vy"
		                #Get vy Speed thanks to the PID controller and checks if in Dead band.
		                vy,vyiMod = self.interfaces.getPIDSpeed(yError
		                                                ,self.interfaces.errYPrevColor
		                                                ,self.interfaces.vyi
		                                                ,self.interfaces.cycle
		                                                ,self.interfaces.kpyColor
		                                                ,self.interfaces.kdyColor
		                                                ,self.interfaces.kiyColor)
		                self.interfaces.vyi=vyiMod
		            #X Controller
		            print "vx"
		            if(abs(xError)<self.interfaces.xDistanceDeadBand):
		                #print "vx en banda de seguridad"
		                vx=0
		                #self.yDeadZone=True
		            else:
		                #Get vx Speed thanks to the PID controller and checks if in Dead band.
		                vx,vxiMod = self.interfaces.getPIDSpeed(xError
		                                                ,self.interfaces.errXPrevColor
		                                                ,self.interfaces.vxi
		                                                ,self.interfaces.cycle
		                                                ,self.interfaces.kpxColor
		                                                ,self.interfaces.kdxColor
		                                                ,self.interfaces.kixColor)
		                self.interfaces.vxi=vxiMod
		        else:
		            centerImage=input_imageData
		            momentsCombined = cv2.moments(maskPrimary+maskSecondary)
		            if(momentsCombined['m00']>0):
		                yCombined = momentsCombined['m10']/momentsCombined['m00']
		                xCombined = momentsCombined['m01']/momentsCombined['m00']
		                yError = self.interfaces.center[1]-yCombined
		                xError = self.interfaces.center[0]-xCombined
		                #Y Controller
		                print "vy"
		                if(abs(yError)<self.interfaces.yDistanceDeadBand):
		                    #print "Vy en banda de seguridad"
		                    vy=0
		                    #self.yDeadZone=True
		                else:
		                    #Get vy Speed thanks to the PID controller and checks if in Dead band.
		                    vy,vyiMod = self.interfaces.getPIDSpeed(yError
		                                                    ,self.interfaces.errYPrevColor
		                                                    ,self.interfaces.vyi
		                                                    ,self.interfaces.cycle
		                                                    ,self.interfaces.kpyColor
		                                                    ,self.interfaces.kdyColor
		                                                    ,self.interfaces.kiyColor)
		                    self.interfaces.vyi=vyiMod
		                #X Controller
		                print "vx"
		                if(abs(xError)<self.interfaces.xDistanceDeadBand):
		                    #print "vx en banda de seguridad"
		                    vx=0
		                    #self.yDeadZone=True
		                else:
		                    #Get vx Speed thanks to the PID controller and checks if in Dead band.
		                    vx,vxiMod = self.interfaces.getPIDSpeed(xError
		                                                    ,self.interfaces.errXPrevColor
		                                                    ,self.interfaces.vxi
		                                                    ,self.interfaces.cycle
		                                                    ,self.interfaces.kpxColor
		                                                    ,self.interfaces.kdxColor
		                                                    ,self.interfaces.kixColor)
		                    self.interfaces.vxi=vxiMod
		elif(areaPrimary>0 and areaSecondary==0):
		    print "Only Primary"
		    yPrimary = momentsPrimary['m10']/momentsPrimary['m00']
		    xPrimary = momentsPrimary['m01']/momentsPrimary['m00']
		    yError = self.interfaces.center[1]-yPrimary
		    xError = self.interfaces.center[0]-xPrimary
		    #Y Controller
		    print "vy"
		    if(abs(yError)<self.interfaces.yDistanceDeadBand):
		        #print "Vy en banda de seguridad"
		        vy=0
		        #self.yDeadZone=True
		    else:
		        #Get vy Speed thanks to the PID controller and checks if in Dead band.
		        vy,vyiMod = self.interfaces.getPIDSpeed(yError
		                                        ,self.interfaces.errYPrevColor
		                                        ,self.interfaces.vyi
		                                        ,self.interfaces.cycle
		                                        ,self.interfaces.kpyColor
		                                        ,self.interfaces.kdyColor
		                                        ,self.interfaces.kiyColor)
		        self.interfaces.vyi=vyiMod
		    #X Controller
		    print "vx"
		    if(abs(xError)<self.interfaces.xDistanceDeadBand):
		        #print "vx en banda de seguridad"
		        vx=0
		        #self.yDeadZone=True
		    else:
		        #Get vx Speed thanks to the PID controller and checks if in Dead band.
		        vx,vxiMod = self.interfaces.getPIDSpeed(xError
		                                        ,self.interfaces.errXPrevColor
		                                        ,self.interfaces.vxi
		                                        ,self.interfaces.cycle
		                                        ,self.interfaces.kpxColor
		                                        ,self.interfaces.kdxColor
		                                        ,self.interfaces.kixColor)
		elif(areaPrimary==0 and areaSecondary>0):
		    print "Only Secondary"
		    ySecondary = momentsSecondary['m10']/momentsSecondary['m00']
		    xSecondary = momentsSecondary['m01']/momentsSecondary['m00']
		    yError = self.interfaces.center[1]-ySecondary
		    xError = self.interfaces.center[0]-xSecondary
		    #Y Controller
		    print "vy"
		    if(abs(yError)<self.interfaces.yDistanceDeadBand):
		        #print "Vy en banda de seguridad"
		        vy=0
		        #self.yDeadZone=True
		    else:
		        #Get vy Speed thanks to the PID controller and checks if in Dead band.
		        vy,vyiMod = self.interfaces.getPIDSpeed(yError
		                                        ,self.interfaces.errYPrevColor
		                                        ,self.interfaces.vyi
		                                        ,self.interfaces.cycle
		                                        ,self.interfaces.kpyColor
		                                        ,self.interfaces.kdyColor
		                                        ,self.interfaces.kiyColor)
		        self.interfaces.vyi=vyiMod
		    #X Controller
		    print "vx"
		    if(abs(xError)<self.interfaces.xDistanceDeadBand):
		        #print "vx en banda de seguridad"
		        vx=0
		        #self.yDeadZone=True
		    else:
		        #Get vx Speed thanks to the PID controller and checks if in Dead band.
		        vx,vxiMod = self.interfaces.getPIDSpeed(xError
		                                        ,self.interfaces.errXPrevColor
		                                        ,self.interfaces.vxi
		                                        ,self.interfaces.cycle
		                                        ,self.interfaces.kpxColor
		                                        ,self.interfaces.kdxColor
		                                        ,self.interfaces.kixColor)
		else:
		   print "Color Beacon not found"
		
		#Function in order to check if the tracking is stable on the center
		if (foundCenter):
		    diff=time.time()-self.interfaces.stableTime
		    if (self.interfaces.stableTime==0):
		        self.interfaces.stableTime=time.time()
		    elif (diff<self.interfaces.timeLimitToLand):
		        print "On target for: "+str(diff)+" seconds"
		    elif (diff>=self.interfaces.timeLimitToLand):
		        print "Changing State"
		        self.interfaces.timeToApril=True   
		else:
		    self.interfaces.stableTime=0
		#Limiting the speed prevents from overshooting as a failsafe    
		vy=self.interfaces.limitSpeed(vy)
		vx=self.interfaces.limitSpeed(vx)            
		#Send orders to ArDrone 2                    
		logging.info("vx=%.4f;" % vx + "vy=%.4f" % vy)
		self.interfaces.myCmdvel.sendCMDVel(vx,vy,0,0,0,0) 
		self.interfaces.errXPrevColor=xError
		self.interfaces.errYPrevColor=yError           
		
		print ""
		print "******************************"
		print ""
		if(self.interfaces.timeToApril):
		    cv2.destroyAllWindows()
		else:
		    cv2.imshow("Target",centerImage)
		#End Takeoff

class State20(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		if(self.interfaces.calibration):
		    #Color beacon initialization
		    tree = ET.parse(self.interfaces.xmlFileName)
		    root = tree.getroot()
		    for colour in root.findall('colour'):
		        name = colour.get('name')
		        if (name=="Primary"):
		            self.interfaces.PH_max = int(colour.find('Hmax').text)
		            self.interfaces.PS_max = int(colour.find('Smax').text)
		            self.interfaces.PV_max = int(colour.find('Vmax').text)
		            self.interfaces.PH_min = int(colour.find('Hmin').text)
		            self.interfaces.PS_min = int(colour.find('Smin').text)
		            self.interfaces.PV_min = int(colour.find('Vmin').text)
		            self.interfaces.PErode = int(colour.find('Erosion').text)
		            self.interfaces.PDilate = int(colour.find('Dilation').text)
		
		        elif(name=="Secondary"):
		            self.interfaces.SH_max = int(colour.find('Hmax').text)
		            self.interfaces.SS_max = int(colour.find('Smax').text)
		            self.interfaces.SV_max = int(colour.find('Vmax').text)
		            self.interfaces.SH_min = int(colour.find('Hmin').text)
		            self.interfaces.SS_min = int(colour.find('Smin').text)
		            self.interfaces.SV_min = int(colour.find('Vmin').text)
		            self.interfaces.SErode = int(colour.find('Erosion').text)
		            self.interfaces.SDilate = int(colour.find('Dilation').text)
		    #Apriltags Initialization
		    HALF_MARKER_SIZE=self.interfaces.MARKER_SIZE/2 
		    self.interfaces.m_MarkerPoints[0,0] =-HALF_MARKER_SIZE
		    self.interfaces.m_MarkerPoints[0,1] = HALF_MARKER_SIZE
		    self.interfaces.m_MarkerPoints[0,2] = 0
		    self.interfaces.m_MarkerPoints[1,0] = HALF_MARKER_SIZE
		    self.interfaces.m_MarkerPoints[1,1] = HALF_MARKER_SIZE
		    self.interfaces.m_MarkerPoints[1,2] = 0
		    self.interfaces.m_MarkerPoints[2,0] = HALF_MARKER_SIZE
		    self.interfaces.m_MarkerPoints[2,1] =-HALF_MARKER_SIZE
		    self.interfaces.m_MarkerPoints[2,2] = 0
		    self.interfaces.m_MarkerPoints[3,0] = -HALF_MARKER_SIZE
		    self.interfaces.m_MarkerPoints[3,1] = -HALF_MARKER_SIZE
		    self.interfaces.m_MarkerPoints[3,2] = 0
		    #self.interfaces.cameraMatrix =  np.resize(self.interfaces.cameraMatrix,3,3)
		    self.interfaces.calibration = False
			
		    print "calibration from file "+ self.interfaces.xmlFileName + " completed"
		
		if(self.interfaces.initialize):
		    input_image = self.interfaces.myCamera.getImage()
		    self.interfaces.initialize=False #Cambiar
		    self.interfaces.center[0] = input_image.height/2 #X axis
		    self.interfaces.center[1] = input_image.width/2  #Y axis
		    self.interfaces.initialize = False
		#Ready to Take Off
		#self.interfaces.myExtra.takeoff()

class State21(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		xSpeed=0
		wSpeed=0
		
		input_image = self.interfaces.myCamera.getImage()
		input_imageData = input_image.data
		centerImage = input_imageData
		hsv = cv2.cvtColor(input_imageData, cv2.COLOR_BGR2HSV)
		maskPrimary,maskRGBPrimary = self.interfaces.hsvFilter(self.interfaces.PH_max,self.interfaces.PS_max,self.interfaces.PV_max,self.interfaces.PH_min,self.interfaces.PS_min,self.interfaces.PV_min,input_image.data,hsv)
		
		maskSecondary,maskRGBSecondary = self.interfaces.hsvFilter(self.interfaces.SH_max,self.interfaces.SS_max,self.interfaces.SV_max,self.interfaces.SH_min,self.interfaces.SS_min,self.interfaces.SV_min,input_image.data,hsv)
		
		kernel = np.ones((3,3),np.uint8)
		
		maskRGBPrimary = cv2.erode(maskRGBPrimary,kernel,iterations = self.interfaces.PErode)
		maskRGBPrimary = cv2.dilate(maskRGBPrimary,kernel,iterations = self.interfaces.PDilate)
		momentsPrimary = cv2.moments(maskPrimary)
		areaPrimary = momentsPrimary['m00']
		
		maskRGBSecondary = cv2.erode(maskRGBSecondary,kernel,iterations = self.interfaces.SErode)
		maskRGBSecondary = cv2.dilate(maskRGBSecondary,kernel,iterations = self.interfaces.SDilate)
		momentsSecondary = cv2.moments(maskSecondary)
		areaSecondary = momentsSecondary['m00']
		
		
		if(areaPrimary>0 or areaSecondary>0):
		    self.interfaces.colorBeaconFound=True
		    self.interfaces.myCmdvel.sendCMDVel(-self.interfaces.searchTrips*self.interfaces.searchSpeed,0,0,0,0,-self.interfaces.searchTrips*self.interfaces.searchSpeed)
		else:
		    if(self.interfaces.cycleCounter>self.interfaces.cycleFreq):
		        self.interfaces.searchTrips+=1
		        self.interfaces.cycleCounter=0
		        self.interfaces.searchSpeed+=self.interfaces.searchIncrement
		    xSpeed=self.interfaces.searchSpeed
		    wSpeed=self.interfaces.searchSpeed*25        
		    self.interfaces.cycleCounter+=1
		    
		self.interfaces.myCmdvel.sendCMDVel(xSpeed,0,0,0,0,wSpeed)

class State23(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		logging.basicConfig(filename='color1',format='%(asctime)s %(message)s',level=logging.INFO,filemode='w')
		start_time = datetime.datetime.now()
		
		vx=0
		vy=0
		xError=0
		yError=0
		foundCenter=False
		#Image Processing
		input_image = self.interfaces.myCamera.getImage()
		input_imageData = input_image.data
		centerImage = input_imageData
		hsv = cv2.cvtColor(input_imageData, cv2.COLOR_BGR2HSV)
		maskPrimary,maskRGBPrimary = self.interfaces.hsvFilter(self.interfaces.PH_max,self.interfaces.PS_max,self.interfaces.PV_max,self.interfaces.PH_min,self.interfaces.PS_min,self.interfaces.PV_min,input_image.data,hsv)
		
		maskSecondary,maskRGBSecondary = self.interfaces.hsvFilter(self.interfaces.SH_max,self.interfaces.SS_max,self.interfaces.SV_max,self.interfaces.SH_min,self.interfaces.SS_min,self.interfaces.SV_min,input_image.data,hsv)
		
		kernel = np.ones((3,3),np.uint8)
		
		maskRGBPrimary = cv2.erode(maskRGBPrimary,kernel,iterations = self.interfaces.PErode)
		maskRGBPrimary = cv2.dilate(maskRGBPrimary,kernel,iterations = self.interfaces.PDilate)
		momentsPrimary = cv2.moments(maskPrimary)
		areaPrimary = momentsPrimary['m00']
		
		maskRGBSecondary = cv2.erode(maskRGBSecondary,kernel,iterations = self.interfaces.SErode)
		maskRGBSecondary = cv2.dilate(maskRGBSecondary,kernel,iterations = self.interfaces.SDilate)
		momentsSecondary = cv2.moments(maskSecondary)
		areaSecondary = momentsSecondary['m00']
		
		maskRGBTot = maskRGBPrimary+maskRGBSecondary
		
		#Controller based on how many tiles have been detected
		if(areaPrimary>0 and areaSecondary>0):
		    print "Both colors detected"
		    crosshair,f = self.interfaces.findCrosshair(input_imageData,maskRGBPrimary,maskRGBSecondary)
		    print "F: %d" % len(f)
		    #f represents the number of cuadrants detected
		    if(len(f)>0):
		        #Returns Center of crossair and checks if the center of the crossair was found.        
		        crossY,crossX,foundCenter,centerImage = self.interfaces.getCrosshairCoord(crosshair,centerImage)
		        logging.info("y:%.2f;"% crossY + "x:%.2f" % crossX)
		        if(foundCenter):
		            print "Center Found!"
		            yError=self.interfaces.center[1]-crossY
		            xError=self.interfaces.center[0]-crossX 
		            #Y Controller
		            if(abs(yError)<self.interfaces.yDistanceDeadBand):
		                #print "Vy en banda de seguridad"
		                vy=0
		                #self.yDeadZone=True
		            else:
		                print "vy"
		                #Get vy Speed thanks to the PID controller and checks if in Dead band.
		                vy,vyiMod = self.interfaces.getPIDSpeed(yError
		                                                ,self.interfaces.errYPrevColor
		                                                ,self.interfaces.vyi
		                                                ,self.interfaces.cycle
		                                                ,self.interfaces.kpyColor
		                                                ,self.interfaces.kdyColor
		                                                ,self.interfaces.kiyColor)
		                self.interfaces.vyi=vyiMod
		            #X Controller
		            print "vx"
		            if(abs(xError)<self.interfaces.xDistanceDeadBand):
		                #print "vx en banda de seguridad"
		                vx=0
		                #self.yDeadZone=True
		            else:
		                #Get vx Speed thanks to the PID controller and checks if in Dead band.
		                vx,vxiMod = self.interfaces.getPIDSpeed(xError
		                                                ,self.interfaces.errXPrevColor
		                                                ,self.interfaces.vxi
		                                                ,self.interfaces.cycle
		                                                ,self.interfaces.kpxColor
		                                                ,self.interfaces.kdxColor
		                                                ,self.interfaces.kixColor)
		                self.interfaces.vxi=vxiMod
		        else:
		            centerImage=input_imageData
		            momentsCombined = cv2.moments(maskPrimary+maskSecondary)
		            if(momentsCombined['m00']>0):
		                yCombined = momentsCombined['m10']/momentsCombined['m00']
		                xCombined = momentsCombined['m01']/momentsCombined['m00']
		                yError = self.interfaces.center[1]-yCombined
		                xError = self.interfaces.center[0]-xCombined
		                #Y Controller
		                print "vy"
		                if(abs(yError)<self.interfaces.yDistanceDeadBand):
		                    #print "Vy en banda de seguridad"
		                    vy=0
		                    #self.yDeadZone=True
		                else:
		                    #Get vy Speed thanks to the PID controller and checks if in Dead band.
		                    vy,vyiMod = self.interfaces.getPIDSpeed(yError
		                                                    ,self.interfaces.errYPrevColor
		                                                    ,self.interfaces.vyi
		                                                    ,self.interfaces.cycle
		                                                    ,self.interfaces.kpyColor
		                                                    ,self.interfaces.kdyColor
		                                                    ,self.interfaces.kiyColor)
		                    self.interfaces.vyi=vyiMod
		                #X Controller
		                print "vx"
		                if(abs(xError)<self.interfaces.xDistanceDeadBand):
		                    #print "vx en banda de seguridad"
		                    vx=0
		                    #self.yDeadZone=True
		                else:
		                    #Get vx Speed thanks to the PID controller and checks if in Dead band.
		                    vx,vxiMod = self.interfaces.getPIDSpeed(xError
		                                                    ,self.interfaces.errXPrevColor
		                                                    ,self.interfaces.vxi
		                                                    ,self.interfaces.cycle
		                                                    ,self.interfaces.kpxColor
		                                                    ,self.interfaces.kdxColor
		                                                    ,self.interfaces.kixColor)
		                    self.interfaces.vxi=vxiMod
		elif(areaPrimary>0 and areaSecondary==0):
		    print "Only Primary"
		    yPrimary = momentsPrimary['m10']/momentsPrimary['m00']
		    xPrimary = momentsPrimary['m01']/momentsPrimary['m00']
		    yError = self.interfaces.center[1]-yPrimary
		    xError = self.interfaces.center[0]-xPrimary
		    #Y Controller
		    print "vy"
		    if(abs(yError)<self.interfaces.yDistanceDeadBand):
		        #print "Vy en banda de seguridad"
		        vy=0
		        #self.yDeadZone=True
		    else:
		        #Get vy Speed thanks to the PID controller and checks if in Dead band.
		        vy,vyiMod = self.interfaces.getPIDSpeed(yError
		                                        ,self.interfaces.errYPrevColor
		                                        ,self.interfaces.vyi
		                                        ,self.interfaces.cycle
		                                        ,self.interfaces.kpyColor
		                                        ,self.interfaces.kdyColor
		                                        ,self.interfaces.kiyColor)
		        self.interfaces.vyi=vyiMod
		    #X Controller
		    print "vx"
		    if(abs(xError)<self.interfaces.xDistanceDeadBand):
		        #print "vx en banda de seguridad"
		        vx=0
		        #self.yDeadZone=True
		    else:
		        #Get vx Speed thanks to the PID controller and checks if in Dead band.
		        vx,vxiMod = self.interfaces.getPIDSpeed(xError
		                                        ,self.interfaces.errXPrevColor
		                                        ,self.interfaces.vxi
		                                        ,self.interfaces.cycle
		                                        ,self.interfaces.kpxColor
		                                        ,self.interfaces.kdxColor
		                                        ,self.interfaces.kixColor)
		elif(areaPrimary==0 and areaSecondary>0):
		    print "Only Secondary"
		    ySecondary = momentsSecondary['m10']/momentsSecondary['m00']
		    xSecondary = momentsSecondary['m01']/momentsSecondary['m00']
		    yError = self.interfaces.center[1]-ySecondary
		    xError = self.interfaces.center[0]-xSecondary
		    #Y Controller
		    print "vy"
		    if(abs(yError)<self.interfaces.yDistanceDeadBand):
		        #print "Vy en banda de seguridad"
		        vy=0
		        #self.yDeadZone=True
		    else:
		        #Get vy Speed thanks to the PID controller and checks if in Dead band.
		        vy,vyiMod = self.interfaces.getPIDSpeed(yError
		                                        ,self.interfaces.errYPrevColor
		                                        ,self.interfaces.vyi
		                                        ,self.interfaces.cycle
		                                        ,self.interfaces.kpyColor
		                                        ,self.interfaces.kdyColor
		                                        ,self.interfaces.kiyColor)
		        self.interfaces.vyi=vyiMod
		    #X Controller
		    print "vx"
		    if(abs(xError)<self.interfaces.xDistanceDeadBand):
		        #print "vx en banda de seguridad"
		        vx=0
		        #self.yDeadZone=True
		    else:
		        #Get vx Speed thanks to the PID controller and checks if in Dead band.
		        vx,vxiMod = self.interfaces.getPIDSpeed(xError
		                                        ,self.interfaces.errXPrevColor
		                                        ,self.interfaces.vxi
		                                        ,self.interfaces.cycle
		                                        ,self.interfaces.kpxColor
		                                        ,self.interfaces.kdxColor
		                                        ,self.interfaces.kixColor)
		else:
		    print "Color Beacon not found"
		    vy,_ = self.interfaces.getPIDSpeed(self.interfaces.errYPrevColor
		                                        ,0
		                                        ,0
		                                        ,self.interfaces.cycle
		                                        ,self.interfaces.kpyColor
		                                        ,0
		                                        ,0)
		        self.interfaces.vyi=vyiMod
		    vx=,_ =self.interfaces.getPIDSpeed(self.interfaces.errXPrevColor
		                                        ,0
		                                        ,0
		                                        ,self.interfaces.cycle
		                                        ,self.interfaces.kpxColor
		                                        ,0
		                                        ,0)
		        self.interfaces.colorBeaconLost=True
		
		#Function in order to check if the tracking is stable on the center
		if (foundCenter):
		    diff=time.time()-self.interfaces.stableTime
		    if (self.interfaces.stableTime==0):
		        self.interfaces.stableTime=time.time()
		    elif (diff<self.interfaces.timeLimitToLand):
		        print "On target for: "+str(diff)+" seconds"
		    elif (diff>=self.interfaces.timeLimitToLand):
		        print "Changing State"
		        self.interfaces.timeToLand=True   
		else:
		    self.interfaces.stableTime=0
		#Limiting the speed prevents from overshooting as a failsafe    
		vy=self.interfaces.limitSpeed(vy)
		vx=self.interfaces.limitSpeed(vx)            
		#Send orders to ArDrone 2                    
		logging.info("vx=%.4f;" % vx + "vy=%.4f" % vy)
		self.interfaces.myCmdvel.sendCMDVel(vx,vy,0,0,0,0) 
		self.interfaces.errXPrevColor=xError
		self.interfaces.errYPrevColor=yError           
		
		print ""
		print "******************************"
		print ""
		if(self.interfaces.timeToApril):
		    cv2.destroyAllWindows()
		else:
		    cv2.imshow("Target",centerImage)
		#End Takeoff

class Tran28(ConditionalTransition):
	def __init__(self, id, destinationId, interfaces):
		ConditionalTransition.__init__(self, id, destinationId)
		self.interfaces = interfaces

	def checkCondition(self):
		return self.interfaces.switchBeacon

	def runCode(self):
		self.interfaces.switchBeacon=False
		self.interfaces.timeOnTarget=0

class Tran11(ConditionalTransition):
	def __init__(self, id, destinationId, interfaces):
		ConditionalTransition.__init__(self, id, destinationId)
		self.interfaces = interfaces

	def checkCondition(self):
		return self.interfaces.switchBeacon

	def runCode(self):
		self.interfaces.switchBeacon=False

class Tran19(ConditionalTransition):
	def __init__(self, id, destinationId, interfaces):
		ConditionalTransition.__init__(self, id, destinationId)
		self.interfaces = interfaces

	def checkCondition(self):
		return self.interfaces.timeToColor

	def runCode(self):
		self.interfaces.myExtra.toggleCam()
		input_image = self.interfaces.myCamera.getImage()
		self.interfaces.center[0] = input_image.height/2 #X axis
		self.interfaces.center[1] = input_image.width/2  #Y axis
		self.interfaces.myExtra.land()

class Tran14(ConditionalTransition):
	def __init__(self, id, destinationId, interfaces):
		ConditionalTransition.__init__(self, id, destinationId)
		self.interfaces = interfaces

	def checkCondition(self):
		return self.interfaces.timeToApril

	def runCode(self):
		self.interfaces.myExtra.toggleCam()
		self.interfaces.center=[0,0] #Comment this line if using Apriltags

class Tran16(TemporalTransition):

	def runCode(self):
		pass

class Tran31(TemporalTransition):

	def runCode(self):
		pass

class Tran22(ConditionalTransition):
	def __init__(self, id, destinationId, interfaces):
		ConditionalTransition.__init__(self, id, destinationId)
		self.interfaces = interfaces

	def checkCondition(self):
		return self.interfaces.colorBeaconFound

	def runCode(self):
		self.interfaces.colorBeaconFound=False

class Tran23(ConditionalTransition):
	def __init__(self, id, destinationId, interfaces):
		ConditionalTransition.__init__(self, id, destinationId)
		self.interfaces = interfaces

	def checkCondition(self):
		return self.interfaces.colorBeaconLost

	def runCode(self):
		self.interfaces.colorBeaconLost=False
		self.interfaces.searchSpeed=0.01
		self.interfaces.searchTrips=1

class Tran24(ConditionalTransition):
	def __init__(self, id, destinationId, interfaces):
		ConditionalTransition.__init__(self, id, destinationId)
		self.interfaces = interfaces

	def checkCondition(self):
		return self.interfaces.timeToLand

	def runCode(self):
		pass

class Interfaces():
	def __init__(self):
		self.jdrc = None
		self.myCmdvel = None
		self.myCamera = None
		self.myExtra = None
		self.maxSpeed = 0.5 #Real_relativa 0.07 #Cambiar max_speed
		self.cycle = 50.0 #in ms
		#Transitions Variables
		self.timeToLand = False
		self.timeToApril = False
		self.timeToColor = False
		self.colorBeaconLost = False
		self.colorBeaconFound = False
		#Xml Files
		#self.xmlFileName = 'calibration_simulated.xml'
		self.xmlFileName = 'calibration.xml'
		#Image Center
		self.center = [0,0]
		#Search
		self.cycleCounter=0
		self.searchTrips=1
		self.cycleFreq=self.cyclesPerTrip*(1/(self.cycle/1000.0)) #Gets how many cycles are need per second/ Our frequency
		self.xSearchSpeed=0.01
		self.searchIncrement=0.005
		self.wSearchSpeed=0.01
		#Initialize
		self.initialize=True
		self.calibration = True
		#For PIDs
		'''Color Beacon Variables'''
		#PID Constants
		self.kpxColor = 0.25   #RealICS #Real 0.012 #Virtual 0.005
		self.kdxColor = 0 #0.05    #Real ICS #Real  #Virtual 0.016
		self.kixColor = 0.01 #0.000001  #Real ICS #Real #Virtual 0.00004
		self.kpyColor = 0.5   #Real ICS #Real #Virtual 0.0025
		self.kdyColor = 0 #0.05    #Real ICS #Real #Virtual 0.008
		self.kiyColor = 0.01 #0.000001  #Real ICS #Real #Virtual 0.00006
		self.kpzColor = 0
		self.kdzColor = 0
		self.kizColor = 0
		self.xDistanceDeadBand = 0.0 #Virtual 0
		self.yDistanceDeadBand = 0.0 #Virtual 0
		self.zDistanceDeadBand = 0
		self.errYPrevColor=0.0
		self.errXPrevColor=0.0
		#Color Filter
		self.minArea = 700 #Gazebo 700
		self.PH_max = 180
		self.PS_max = 255
		self.PV_max = 255
		self.PH_min = 0
		self.PS_min = 0
		self.PV_min = 0
		self.PErode = 0
		self.PDilate = 0
		self.SH_max = 180
		self.SS_max = 255
		self.SV_max = 255
		self.SH_min = 0
		self.SS_min = 0
		self.SV_min = 0
		self.SErode = 0
		self.SDilate = 0
		self.alpha = 0.6
		self.vxiColor = 0.0
		self.vyiColor = 0.0
		self.vziColor = 0.0
		#Stabilization checking
		self.stableTime=0
		self.timeLimitToLand = 6 #In seconds
		'''Apriltag Beacon Variables'''
		self.dead_band_x = 0 #Real_imagen #Real_pose 0.4  #Real_April 0.1 #Virtual 0.2
		self.dead_band_y = 0.1 #Real_imagen #Real_pose 0.15 #Real_April 0.1 #Virtual 0.2
		self.dead_band_z = 0.2 #Real_imagen #Real 0.2 #Virtual 0.2
		self.min_z = 2.6       #Real 2.6
		self.dead_band_w = 0.09 #Real_imagen #Real 0.180 #0.09
		self.targetCenter = [0,0] #CAMBIAR
		self.kp = 0.01    #Real ICS #Real_pose 0.1 #Real_April 0.0007 #Virtual 0.0025
		self.kd = 0       #Real ICS #Real_pose 1500 #Real_April 0.4 #Virtual 0.008
		self.ki = 0  #Real ICS #Real_pose #Real_April 0.000006 #Virtual 0.00006
		self.kpw = 0.1      #Real ICS #Real 0.1 #Virtual 0.1
		self.kdw = 10        #Real ICS #Real 0 #Virtual 0
		self.kiw = 0.001        #Real ICS #Real 0 #Virtual 0
		self.kpz = 0.006    #Real ICS #Real 0.006 #Virtual 0.1
		self.kdz = 1        #Real ICS #Real 1 | 0.7#Virtual 0.01
		self.kiz = 0.00008  #Real ICS # Real 0.00006# Virtual 0.001
		self.error_xy_anterior = [0.0,0.0]
		self.error_z_anterior = 0.0
		self.error_w_anterior = 0.0
		self.vxi = 0
		self.vyi = 0
		self.vzi = 0
		self.vwi = 0
		#AprilTags
		self.options = apriltag.DetectorOptions()
		self.detector = apriltag.Detector(self.options)
		#Marker Size
		self.MARKER_SIZE = 0.28 #Size in meters
		self.m_MarkerPoints = np.zeros((4,3),dtype=np.float32)
		#Image Processing
		#self.cameraMatrix = np.resize(np.array([187.336, 0, 160, 0, 187.336, 120, 0, 0, 1]),(3,3)) #Gazebo simulator
		self.cameraMatrix = np.resize(np.array([731.37257937739332, 0, 322.35647387552422, 0, 734.23757205576192, 246.30430666269825, 0, 0, 1]),(3,3)) #ArDrone 2
		#Distorsion Coefficients
		#self.distCoeffs = np.zeros((5, 1), np.uint8) #Gazebo
		self.distCoeffs = np.array([-0.07304604105914128, 0.50646907582979650, 0.00024443957708413687, 0.00074556540195940392,-1.0762308065763191]) #ArDrone 2
		#Pose
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
		self.pitch = 0.0
		#Beacons
		self.beacons = [4,7,16,30]
		self.beaconCounter = 0
		self.switchBeacon = False
		self.timeOnTarget=0.0
		self.timeLimitToChange = 10 #In seconds
		'''Performance monitor variables'''
		#Performance monitor
		self.minf = 10000
		self.maxf = 0
		self.avgf = 0
		self.listf = [0,0,0]
		self.c = 0

		self.connectProxies()

	def connectProxies(self):
		cfg = config.load(sys.argv[1])
		self.jdrc = comm.init(cfg, "escenario_combinado_c")
		self.myCmdvel = self.jdrc.getCMDVelClient("escenario_combinado_c.myCmdvel")
		if not self.myCmdvel:
			raise Exception("could not create client with name:myCmdvel")
		print("myCmdvel is connected")
		self.myCamera = self.jdrc.getCameraClient("escenario_combinado_c.myCamera")
		if not self.myCamera:
			raise Exception("could not create client with name:myCamera")
		print("myCamera is connected")
		self.myExtra = self.jdrc.getArDroneExtraClient("escenario_combinado_c.myExtra")
		if not self.myExtra:
			raise Exception("could not create client with name:myExtra")
		print("myExtra is connected")

	def destroyProxies(self):
		if self.jdrc is not None:
			self.jdrc.destroy()

	def rollCorrection(self,t):

	

	    if (t>=0):

	

	        t = (2*t)/(math.pi) - 1

	

	    else:

	

	        t = (2*t)/(math.pi) + 1

	

	        

	

	    return t

	

	def pitchCorrection(self,x,th,pitch):

	    if(pitch<0):

	        if((th-x)<0):

	            pitch=-pitch

	    else:

	        if((th-x)>0):

	            pitch=-pitch

	    return pitch

	

	#Checks if valid Rotation Matrix

	

	def isRotationMatrix(self,R) :

	

	    Rt = np.transpose(R)

	

	    shouldBeIdentity = np.dot(Rt, R)

	

	    I = np.identity(3, dtype = R.dtype)

	

	    n = np.linalg.norm(I - shouldBeIdentity)

	

	    return n < 1e-6

	

	  

	

	#Calculates rotation matrix to euler angles

	

	def rotationMatrixToEulerAngles(self,R) :

	

	 

	

	    assert(self.isRotationMatrix(R))

	

	     

	

	    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

	

	     

	

	    singular = sy < 1e-6

	

	 

	

	    if  not singular :

	

	        x = math.atan2(R[2,1] , R[2,2])

	

	        y = math.atan2(-R[2,0], sy)

	

	        z = math.atan2(R[1,0], R[0,0])

	

	    else :

	

	        x = math.atan2(-R[1,2], R[1,1])

	

	        y = math.atan2(-R[2,0], sy)

	

	        z = 0

	

	 

	

	    return np.array([x, y, z])

	

	

	

	def hsvFilter(self,hup,sup,vup,hdwn,sdwn,vdwn,img,hsv):

	

	    if hdwn <= hup:

	

	            # http://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html

	

	            minValues = np.array([hdwn,sdwn,vdwn],dtype=np.uint8)

	

	            maxValues = np.array([hup,sup,vup], dtype=np.uint8)

	

	

	

	            mask = cv2.inRange(hsv, minValues, maxValues)

	

	            res = cv2.bitwise_and(img,img, mask= mask)

	

	            return mask,res

	

	    else:

	

	        # http://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html

	

	        # red goes from 240 to 20 degreess aprox

	

	        maxValues1 = np.array([hup,sup,vup],dtype=np.uint8)

	

	        minValues1 = np.array([0,sdwn,vdwn],dtype=np.uint8)

	

	        maxValues2 = np.array([180,sup,vup], dtype=np.uint8)

	

	        minValues2 = np.array([hdwn,sdwn,vdwn],dtype=np.uint8)

	

	

	

	        mask1 = cv2.inRange(hsv, minValues1, maxValues1)

	

	        mask2 = cv2.inRange(hsv, minValues2, maxValues2)

	

	        restMask = mask1 + mask2

	

	

	

	        r1 = cv2.bitwise_and(img,img, mask= mask1)

	

	        r2 = cv2.bitwise_and(img,img, mask= mask2)

	

	        res = cv2.bitwise_or(r1, r2)

	

	        return restMask,res

	

	

	

	def drawContours(self,contours,f):

	

	    i=0

	

	    areas = [cv2.contourArea(c) for c in contours]

	

	    for extension in areas:

	

	        print "Area: "+str(extension)

	

	        if extension > self.minArea:

	

	            img = np.zeros((self.center[0]*2,self.center[1]*2,3), np.uint8)

	

	            actual = contours[i]

	

	            approx = cv2.approxPolyDP(actual,0.05*cv2.arcLength(actual,True),True)

	

	            cv2.drawContours(img,[actual],0,(0,30,0),12)            

	

	            f.append(img)            

	

	            i=i+1

	

	    return f

	

	

	

	def findCrosshair(self,input_image,maskRGBPrimary,maskRGBSecondary):

	

	    f=[]

	

	    i=0

	

	    #calculating crosshair between colours

	

	    imgray = cv2.cvtColor(maskRGBPrimary,cv2.COLOR_BGR2GRAY)

	

	    ret,thresh = cv2.threshold(imgray,255,255,255)

	

	    _,contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

	

	    print "Primary"

	

	    f = self.drawContours(contours,f)

	

	    imgray = cv2.cvtColor(maskRGBSecondary,cv2.COLOR_BGR2GRAY)

	

	    ret,thresh = cv2.threshold(imgray,255,255,255)

	

	    _,contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

	

	    print "Secondary"

	

	    f = self.drawContours(contours,f)

	

	    kernel = np.ones((5,5),np.uint8)

	

	    result = input_image

	

	    if(len(f)>0):

	

	        f[0] = cv2.dilate(f[0],kernel,iterations = 4)

	

	        result=f[0]

	

	        for k in range(len(f)-1):

	

	            f[k+1] = cv2.dilate(f[k+1],kernel,iterations = 4)

	

	            result=result+f[k+1]

	

	    return result,f

	

	

	

	def getCrosshairCoord(self, crosshair,image):

	

	    centerDetectedY=0

	

	    centerDetectedX=0

	

	    crossYarr=[]

	

	    crossXarr=[]

	

	    foundCenter=False

	    circleImage = image

	

	    lower_green = np.array([0,80,0], dtype=np.uint8)

	

	    upper_green = np.array([0, 255,0], dtype=np.uint8)

	

	    maskSHI = cv2.inRange(crosshair, lower_green, upper_green)

	

	    #filtering center from rest of the image

	

	    centerImage = cv2.bitwise_and(crosshair,crosshair, mask= maskSHI)

	

	    compare_image = np.zeros((self.center[0]*2,self.center[1]*2,3), np.uint8)

	

	    diff = cv2.absdiff(compare_image, centerImage)

	

	    imgray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)

	

	    _,contours,_ = cv2.findContours(imgray,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

	

	

	

	    if(len(contours)>0):

	

	        foundCenter=True

	

	        for c in contours:

	

	            if(cv2.contourArea(c) >= 0):

	

	                rectY, rectX, width, height = cv2.boundingRect(c)

	

	                cv2.rectangle(circleImage,(rectY,rectX),(rectY+width,rectX+height),(0,0,255),2)

	

	                #calculating center of crossair

	

	                crossY= (rectY+rectY+width)/2

	

	                crossX= (rectX+rectX+height)/2

	

	                #

	

	                #img = show_image+1-1

	

	                radius = int(math.sqrt(((rectY+width)^2)+((rectX+height)^2)))

	

	                cv2.circle(circleImage,(rectY,rectX), radius, (255,255,0), 1)

					

	                crossYarr.append(crossY)

	

	                crossXarr.append(crossX)

	

	        for values in crossYarr:

	

	            centerDetectedY+=values

	

	        centerDetectedY/= len(crossYarr)

	

	        for values in crossXarr:

	

	            centerDetectedX+=values

	

	        centerDetectedX/= len(crossXarr)

	

	    return centerDetectedY,centerDetectedX,foundCenter,circleImage

	

	

	

	#Calculate PID

	

	def getPIDSpeed(self,error,pError,vi,cycle,kp,kd,ki):

	    print vi,cycle

	    #Proporcional

	

	    vp = kp * error

	

	    #Derivada

	

	    vd = kd * ((error-pError)/cycle)

	

	    #Integral 

	    viModified = ki * (vi+(error*cycle))

	

	    print "vp=%.4f" % vp,"vd=%.4f" % vd,"viModified=%.4f" % viModified

	

	    #Total

	

	    result = vp + vd + viModified

	

	    return  result, viModified

	

	

	

	def getPerformance(self,start):

	

	    finish_Time = datetime.datetime.now()

	

	    dt = finish_Time - start_time

	

	    ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0

	

	    return ms #Time in miliseconds

	

	

	def limitSpeed(self,vTarget):    

	    if abs(vTarget)>self.maxSpeed:

	        if vTarget>0:

	            vTarget=self.maxSpeed

	        else:

	            vTarget=-self.maxSpeed  

	    return vTarget

displayGui = False
guiThread = None
gui = None
state0 = None

def signal_handler(signal, frame):
	global gui
	print("SIGINT is captured. The program exits")
	if gui is not None:
		gui.close()
	global state0
	state0.stop()

def readArgs():
	global displayGui
	for arg in sys.argv:
		splitedArg = arg.split('=')
		if splitedArg[0] == '--displaygui':
			if splitedArg[1] == 'True' or splitedArg[1] == 'true':
				displayGui = True
				print('runtime gui enabled')
			else:
				displayGui = False
				print('runtime gui disabled')

def runGui():
	global gui
	app = QApplication(sys.argv)
	gui = RunTimeGui()
	gui.show()
	app.exec_()

if __name__ == "__main__":
	interfaces = Interfaces()

	readArgs()
	if displayGui:
		guiThread = threading.Thread(target=runGui)
		guiThread.start()


	if displayGui:
		while(gui is None):
			time.sleep(0.1)

		gui.addState(0, "root", True, 0.0, 0.0, None)
		gui.addState(1, "April Beacon Tracking", False, 841.0, 1208.0, 0)
		gui.addState(3, "AprilTag Rotational Search", False, 846.0, 974.0, 0)
		gui.addState(4, "Land", False, 1155.0, 1213.0, 0)
		gui.addState(12, "Color Beacon Tracking", False, 846.0, 857.0, 0)
		gui.addState(20, "Initialization and Take Off", True, 847.0, 733.0, 0)
		gui.addState(21, "Spiral Search", False, 1158.0, 974.0, 0)
		gui.addState(23, "Color Beacon Landing", False, 1154.0, 1089.0, 0)

		gui.addTransition(28, "Next April Beacon", 1, 3, 957.5, 1088.0)
		gui.addTransition(11, "Beacon Found", 3, 1, 844.0, 1087.5)
		gui.addTransition(19, "Color Beacon Search", 3, 21, 1002.5, 974.5)
		gui.addTransition(14, "Stable on Color Beacon", 12, 3, 845.5, 911.5)
		gui.addTransition(16, "Controlled Take Off", 20, 12, 846.5, 788.0)
		gui.addTransition(31, "transition 31", 20, 21, 1002.5, 853.5)
		gui.addTransition(22, "Found", 21, 23, 1204.5, 1030.5)
		gui.addTransition(23, "Lost", 23, 21, 1101.5, 1036.5)
		gui.addTransition(24, "Stable on Color Beacon", 23, 4, 1155.0, 1151.0)

	if displayGui:
		gui.emitLoadFromRoot()
		gui.emitActiveStateById(0)

	state0 = State0(0, True, interfaces, 50, None, gui)
	state1 = State1(1, False, interfaces, 100, state0, gui)
	state3 = State3(3, False, interfaces, 100, state0, gui)
	state4 = State4(4, False, interfaces, 100, state0, gui)
	state12 = State12(12, False, interfaces, 100, state0, gui)
	state20 = State20(20, True, interfaces, 100, state0, gui)
	state21 = State21(21, False, interfaces, 100, state0, gui)
	state23 = State23(23, False, interfaces, 100, state0, gui)

	tran28 = Tran28(28, 3, interfaces)
	state1.addTransition(tran28)

	tran11 = Tran11(11, 1, interfaces)
	state3.addTransition(tran11)

	tran19 = Tran19(19, 21, interfaces)
	state3.addTransition(tran19)

	tran14 = Tran14(14, 3, interfaces)
	state12.addTransition(tran14)

	tran16 = Tran16(16, 12, 500)
	state20.addTransition(tran16)

	tran31 = Tran31(31, 21, 100)
	state20.addTransition(tran31)

	tran22 = Tran22(22, 23, interfaces)
	state21.addTransition(tran22)

	tran23 = Tran23(23, 21, interfaces)
	state23.addTransition(tran23)

	tran24 = Tran24(24, 4, interfaces)
	state23.addTransition(tran24)

	try:
		state0.startThread()
		signal.signal(signal.SIGINT, signal_handler)
		signal.pause()
		state0.join()
		if displayGui:
			guiThread.join()

		interfaces.destroyProxies()
	except:
		state0.stop()
		if displayGui:
			gui.close()
			guiThread.join()

		state0.join()
		interfaces.destroyProxies()
		sys.exit(1)

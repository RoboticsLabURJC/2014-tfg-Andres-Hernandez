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
		print "*****"
		
		vx=0
		vy=0
		vz=0
		vw=0
		
		#Generation of Apriltag Detector
		
		
		#Algorithm
		#Image Processing
		inputImage = self.interfaces.myCamera.getImage()
		inputImageData = inputImage.data
		inputImageData_h = inputImage.height
		inputImageData_w = inputImage.width
		inputImageData.shape = inputImageData_h, inputImageData_w, 3
		color = cv2.cvtColor(inputImageData, cv2.COLOR_RGB2BGR)
		#Apriltags Detection
		gray = cv2.cvtColor(inputImageData, cv2.COLOR_RGB2GRAY)
		cv2.imshow("Image",gray)
		cv2.waitKey(30)
		detections = self.interfaces.detector.detect(gray)
		num_detections = len(detections)
		if (num_detections <= 0):
		    self.interfaces.x = 0.0
		    self.interfaces.y = 0.0
		    self.interfaces.z = 0.0
		    self.interfaces.pitch = 0.0
		else:
		    for i, detection in enumerate(detections):
		        if (detection.tag_id == self.interfaces.beacons[self.interfaces.beaconCounter]):
		            #print "Beacon %d detected" % detection.tag_id
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
		            retVal,rvec,tvec =cv2.solvePnP(m_MarkerPoints,detection.corners,cameraMatrix,distCoeffs)
		            rodri = cv2.Rodrigues(rvec)
		            #We get X,Y and Z
		            cameraPosition = -np.matrix(rodri[0]).T * np.matrix(tvec)
		            #self.interfaces.x = (detection.center[1] - (inputImageData_h/2))/100 
		            #self.interfaces.y = (detection.center[0] - (inputImageData_w/2))/100
		            self.interfaces.x = cameraPosition.item(0)
		            self.interfaces.y = cameraPosition.item(1)
		            self.interfaces.z = cameraPosition.item(2)
		            #We get roll, pitch and yaw from Euler Angles        
		            eulerAngles = self.interfaces.rotationMatrixToEulerAngles(rodri[0])
		            #yaw = eulerAngles[0]
		            self.interfaces.pitch = self.interfaces.pitchCorrection(eulerAngles.item(0))
		            #pitch = eulerAngles[1]
		            #roll = self.interfaces.rollCorrection(eulerAngles[2])
		            #print yaw,pitch,roll
		        else:  
		            print "Beacon %d not detected" % self.interfaces.beacons[self.interfaces.beaconCounter] 
		            self.interfaces.x = 0.0
		            self.interfaces.y = 0.0
		            self.interfaces.z = 0.0
		            self.interfaces.pitch = 0.0
		        
		    print ("x %.3f" % self.interfaces.x,"y %.3f" % self.interfaces.y, "z %.3f" % self.interfaces.z,"pitch %.3f" % self.interfaces.pitch)
		    if (abs(self.interfaces.x)>0 and abs(self.interfaces.y)>0 and abs(self.interfaces.z)>0 and abs(self.interfaces.pitch)>0):
		        
		        #PID controller
		        error_xy = [self.interfaces.center[0]-self.interfaces.x, self.interfaces.center[1]-self.interfaces.y]
		        error_z = self.interfaces.min_z - self.interfaces.z
		        error_w = -self.interfaces.pitch
		        #print ("error_x %.4f" % error_xy[0],"error_y %.4f" % error_xy[1], "error_z %.4f" % error_z,"error_pitch %.4f" % error_w)
		        #Dead Bands
		        #VX
		        if abs(error_xy[0])<self.interfaces.dead_band_x:
		            vx=0
		        else:
		            #Proporcional
		            vxp = self.interfaces.kp * error_xy[0]
		            #Derivada
		            vxd = self.interfaces.kd * ((error_xy[0]-self.interfaces.error_xy_anterior[0])/self.interfaces.cycle)
		            #Integral 
		            self.interfaces.vxi = self.interfaces.ki * (self.interfaces.vxi+(error_xy[0]*self.interfaces.cycle))
		            #Total
		            vx = vxp + vxd + self.interfaces.vxi
		            #print vxp,vxd,self.interfaces.vxi
		        #VY
		        if abs(error_xy[1])<self.interfaces.dead_band_y:
		            vy=0
		        else:
		            #Proporcional
		            vyp = self.interfaces.kp * error_xy[1]
		            #Derivada
		            vyd = self.interfaces.kd * ((error_xy[1]-self.interfaces.error_xy_anterior[1])/self.interfaces.cycle)
		            #Integral
		            self.interfaces.vyi = self.interfaces.ki * (self.interfaces.vyi+(error_xy[1]*self.interfaces.cycle))
		            #Total
		            print self.interfaces.vyi            
		            vy = vyp + vyd + self.interfaces.vyi
		        #VZ
		        if abs(error_z)<self.interfaces.dead_band_z:
		            vz=0
		        else:
		            #Proporcional
		            vzp = self.interfaces.kpz * error_z
		            #Derivada
		            vzd = self.interfaces.kdz * ((error_z-self.interfaces.error_z_anterior)/self.interfaces.cycle)
		            #Integral
		            self.interfaces.vzi = self.interfaces.kiz * (self.interfaces.vzi+(error_z*self.interfaces.cycle))
		            #Total
		            vz = vzp + vzd + self.interfaces.vzi
		
		        #VW
		        if abs(error_w)<self.interfaces.dead_band_w:
		            vw=0
		        else:
		            #Proporcional
		            vwp = self.interfaces.kpw * error_w
		            #Derivada
		            vwd = self.interfaces.kdw * ((error_w-self.interfaces.error_w_anterior)/self.interfaces.cycle)
		            #Integral
		            self.interfaces.vwi = self.interfaces.kiw * (self.interfaces.vwi+(error_w*self.interfaces.cycle))
		            #Total
		            vw = vwp + vwd + self.interfaces.vwi        
		
		        #Max_speed
		        if abs(vx)>self.interfaces.max_speed:
		            if vx<0:
		                vx=-self.interfaces.max_speed
		            else:
		                vx=self.interfaces.max_speed
		        if abs(vy)>self.interfaces.max_speed:
		            if vy<0:
		                vy=-self.interfaces.max_speed
		            else:
		                vy=self.interfaces.max_speed
		        if abs(vz)>self.interfaces.max_speed:
		            if vz<0:
		                vz=-self.interfaces.max_speed
		            else:
		                vz=self.interfaces.max_speed
		        if abs(vw)>self.interfaces.max_speed:
		            if vw<0:
		                vw=-self.interfaces.max_speed
		            else:
		                vw=self.interfaces.max_speed
		        if(vx==0):
		            self.interfaces.error_xy_anterior[0] = 0
		        else:
		            self.interfaces.error_xy_anterior[0] = error_xy[0]
		        if(vy==0):
		            self.interfaces.error_xy_anterior[1] = 0
		        else:
		            self.interfaces.error_xy_anterior[1] = error_xy[1]
		        if(vy==0):
		            self.interfaces.error_z_anterior = 0
		        else:
		            self.interfaces.error_z_anterior = error_z   
		        if(vy==0):
		            self.interfaces.error_w_anterior = 0
		        else:
		            self.interfaces.error_w_anterior = error_w       
		    else:
		        self.interfaces.error_xy_anterior = [0.0,0.0]
		        self.interfaces.error_z_anterior = 0.0
		        self.interfaces.error_w_anterior = 0.0
		        self.interfaces.vxi = 0.0
		        self.interfaces.vyi = 0.0
		        self.interfaces.vzi = 0.0
		        self.interfaces.vwi = 0.0
		
		    #Print Speeds
		    print ("vz:%.3f" % -vz,"vy:%.3f" % -vy,"vx:%.3f" % -vx,"vw:%.3f" % vw)
		    print "*****"
		    #Send Speeds
		    self.interfaces.myCmdvel.sendCMDVel(-vz,-vy,-vx,0,0,vw)

class State3(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		if self.interfaces.beaconCounter >= (len(self.interfaces.beacons)):
		    #Landing Time
		    self.interfaces.timeToLand = True
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
		    #cv2.imshow("Image",gray)
		    #cv2.waitKey(30)
		    detections = self.interfaces.detector.detect(gray)
		    num_detections = len(detections)
		    #print self.interfaces.beacons[self.interfaces.beaconCounter]
		    for i, detection in enumerate(detections):
		        if (detection.tag_id == self.interfaces.beacons[self.interfaces.beaconCounter]):
		            self.interfaces.switchBeacon=True

class State4(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		self.interfaces.myExtra.land()

class State11(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		if(self.interfaces.switchBeacon):
		    self.interfaces.beaconCounter += 1
		    self.interfaces.switchBeacon = False
		

class State12(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		start_time = datetime.datetime.now()
		
		vx=0
		vy=0
		xError=0
		yError=0
		
		if(self.interfaces.calibration):
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
		    print "calibration from file "+ self.interfaces.xmlFileName + " completed"
		    self.interfaces.calibration = False
		
		input_image = self.interfaces.myCamera.getImage()
		input_imageData = input_image.data
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
		
		if(self.interfaces.initialize):
		    #self.interfaces.initialTime=time.time()
		    self.interfaces.initialize=False #Cambiar
		    #self.interfaces.centroImagen(input_image, hsv)
		    self.interfaces.center[0] = input_image.height/2 #X axis
		    self.interfaces.center[1] = input_image.width/2  #Y axis
		    #self.interfaces.myDroneExtra.takeoff()
		
		#Try to get crosshair image if close enough
		if(areaPrimary>0 and areaSecondary>0):
		    crosshair,f = self.interfaces.findCrosshair(input_imageData,maskRGBPrimary,maskRGBSecondary)
		    print "F: %d" % len(f)
		    #f represents the number of cuadrants detected
		    if(len(f)>0):
		        #Returns Center of crossair and image with Rectangle surrounding the center of the crossair.
		        #Coordinate system is following Top-left corner. Translation of coordinates needed.
		        crossY,crossX,foundCenter = self.interfaces.getCrosshairCoord(crosshair)
		        if(foundCenter):
		            #Y Controller
		            if(crossY>0):
		                yError = self.interfaces.center[1]-crossY
		                #Proportional controller
		                if(abs(yError)<self.interfaces.yDistanceDeadZone):
		                    #print "Vy en banda de seguridad"
		                    vy=0
		                    #self.interfaces.yDeadZone=True
		                else:
		                    vy,self.interfaces.vyi = self.interfaces.getPIDSpeed(yError
		                                                    ,self.interfaces.errYPrevColor
		                                                    ,self.interfaces.vyi,self.interfaces.cycle
		                                                    ,self.interfaces.kpyColor,self.interfaces.kdyColor
		                                                    ,self.interfaces.kiyColor)
		                    self.interfaces.vyi=self.interfaces.vyi
		                    #self.interfaces.yDeadZone = False
		            else:
		                vy=0
		            #X Controller
		            if(crossX>0):
		                xError = self.interfaces.center[0]-crossX        
		                if(abs(xError)<self.interfaces.xDistanceDeadZone):
		                    #print "Vx en banda de seguridad"
		                    vx=0
		                    #self.interfaces.xDeadZone=True
		                else:
		                    vx,self.interfaces.vxi = self.interfaces.getPIDSpeed(xError
		                                                    ,self.interfaces.errXPrevColor
		                                                    ,self.interfaces.vxi,self.interfaces.cycle
		                                                    ,self.interfaces.kpxColor,self.interfaces.kdxColor
		                                                    ,self.interfaces.kixColor)
		                    #self.interfaces.xDeadZone=False
		                '''
		                if (self.interfaces.yDeadZone and self.interfaces.xDeadZone):
		                    diff=time.time()-self.interfaces.stableTime
		                    if (self.interfaces.stableTime==0):
		                        self.interfaces.stableTime=time.time()
		                    elif (diff<self.interfaces.timeLimitToLand):
		                        print "On target for: "+str(diff)+" seconds"
		                    elif (diff>=self.interfaces.timeLimitToLand):
		                        self.interfaces.timeToLand=True  
		                else:
		                    self.interfaces.stableTime=0
		                '''
		                if abs(vx)>self.interfaces.maxSpeed:
		                    #print "Vx max speed reached("+str(vx)+")"
		                    if vx>0:
		                        vx=self.interfaces.maxSpeed
		                    else:
		                        vx=-self.interfaces.maxSpeed          
		                if abs(vy)>self.interfaces.maxSpeed:
		                    #print "Vy max speed reached("+str(vy)+")"
		                    if vy>0:
		                        vy=self.interfaces.maxSpeed
		                    else:
		                        vy=-self.interfaces.maxSpeed
		            else:
		                vx=0
		        else:
		            print "acercar a colores"
		elif(areaPrimary>0 and areaSecondary==0):
		    print "Only Primary"
		elif(areaPrimary==0 and areaSecondary>0):
		    print "Only Secondary"
		else:
		   print "Color Beacon Lost"
		self.interfaces.errXPrevColor=xError
		self.interfaces.errYPrevColor=yError
		#Send orders to ArDrone 2                    
		print "vx=%.4f " % vx, "vy=%.4f " % vy
		self.interfaces.myCmdvel.sendCMDVel(vx,vy,0,0,0,0)            
		
		print ""
		print "******************************"
		print ""
		
		#End Takeoff

class Tran12(TemporalTransition):

	def runCode(self):
		pass

class Tran9(ConditionalTransition):
	def __init__(self, id, destinationId, interfaces):
		ConditionalTransition.__init__(self, id, destinationId)
		self.interfaces = interfaces

	def checkCondition(self):
		return self.interfaces.timeToLand

	def runCode(self):
		pass

class Tran11(ConditionalTransition):
	def __init__(self, id, destinationId, interfaces):
		ConditionalTransition.__init__(self, id, destinationId)
		self.interfaces = interfaces

	def checkCondition(self):
		return self.interfaces.switchBeacon

	def runCode(self):
		pass

class Tran13(ConditionalTransition):
	def __init__(self, id, destinationId, interfaces):
		ConditionalTransition.__init__(self, id, destinationId)
		self.interfaces = interfaces

	def checkCondition(self):
		return not self.interfaces.switchBeacon

	def runCode(self):
		pass

class Interfaces():
	def __init__(self):
		self.jdrc = None
		self.myCmdvel = None
		self.myCamera = None
		self.myExtra = None
		self.maxSpeed = 2 #Real_relativa 0.07 #Cambiar max_speed
		#Transitions Variables
		self.timeToLand = False
		self.timeToApril = False
		self.timeToColor = False
		#Xml Files
		self.xmlFileName = 'calibration_simulated.xml'
		#self.xmlFileName = 'calibration.xml'
		#Image Center
		self.center = [0,0]
		#For PIDs
		self.cycle = 50 #in ms
		'''Color Beacon Variables'''
		#PID Constants
		self.kpxColor = 0.005 #0.012 #Virtual 0.005
		self.kdxColor = 0.016 #Virtual 0.01
		self.kixColor = 0.00004 #Virtual 0.00006
		self.kpyColor = 0.0025 #Virtual
		self.kdyColor = 0.008 #Virtual
		self.kiyColor = 0.00006 #Virtual
		self.kpzColor = 0 #Virtual
		self.kdzColor = 0 #Virtual
		self.kizColor = 0 #Virtual
		self.initialize=True
		self.stableTime=0
		self.errYPrevColor=0
		self.errXPrevColor=0
		self.yanteriorTot=0
		self.xanteriorTot=0
		#self.x_img=0
		#self.y_img=0
		#self.landed=True #Cambiar
		#self.turnland=0 #Cambiar
		#self.numIteracionesOrange=0 #Revisar
		#self.numIteracionesGreen=0 #Revisar
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
		self.vxiColor = 0
		self.vyiColor = 0
		self.vziColor = 0
		self.xDistanceDeadZone = 0 #Virtual 0
		self.yDistanceDeadZone = 0 #Virtual 0
		self.xDeadZone = False
		self.yDeadZone = False
		self.calibration = True
		self.timeLimitToLand = 3 #In seconds
		'''Apriltag Beacon Variables'''
		self.dead_band_x = 0.2 #Real_imagen #Real_relativas 0.4 #Virtual 0.2
		self.dead_band_y = 0.15 #Real_imagen #Real_relativas 0.15 #Virtual 0.2
		self.dead_band_z = 0.3 #Real_imagen #Real_relativas 0.3 #Virtual 0.2
		self.min_z = 2.8
		self.dead_band_w = 0.230 #Real_imagen #Real_relativas 0.230 #0.2
		self.targetCenter = [0,0] #CAMBIAR
		self.kp = 0.1 #Real_relativas 0.1 #Virtual 0.1
		self.kd = 1500 #Real_imagen 0.01 #Real_relativas 0.7 #Virtual 0.03
		self.ki = 0.01 #Real_imangen #Real_relativas0.01 #Virtual 0.005
		self.kpw = 0.1 #Virtual 0.1
		self.kdw = 1500 #Real 0.7 #Virtual 0
		self.kiw = 0.01 #Real 0.01 #Virtual 0
		self.kpz = 0.4 #Real 0.4 #Virtual 0.1
		self.kdz = 1500 #Real 0.7#Virtual 0.01
		self.kiz = 0.01 # Real 0.01# Virtual 0.001
		self.error_xy_anterior = [0,0]
		self.error_z_anterior = 0
		self.error_w_anterior = 0
		self.vxi = 0
		self.vyi = 0
		self.vzi = 0
		self.vwi = 0
		#AprilTags
		self.options = apriltag.DetectorOptions()
		self.detector = apriltag.Detector(self.options)
		self.MARKER_SIZE = 0.28
		#Image Processing
		self.cameraMatrix = [187.336, 0, 160, 0, 187.336, 120, 0, 0, 1] #Gazebo simulator
		#self.cameraMatrix = [731.37257937739332, 0, 322.35647387552422, 0, 734.23757205576192, 246.30430666269825, 0, 0, 1] #ArDrone 2
		#Distorsion Coefficients
		self.distCoeffs = np.zeros((5, 1), np.uint8) #Gazebo
		#self.distCoeffs = np.array([-0.07304604105914128, 0.50646907582979650, 0.00024443957708413687, 0.00074556540195940392,-1.0762308065763191]) #ArDrone 2
		#Pose
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
		self.pitch = 0.0
		#Beacons
		self.beacons = [4,7]
		self.beaconCounter = 0
		self.switchBeacon = False
		#Performance monitor
		self.minf = 10000
		self.maxf = 0
		self.avgf = 0
		self.listf = [0,0,0]
		self.c = 0

		self.connectProxies()

	def connectProxies(self):
		cfg = config.load(sys.argv[1])
		self.jdrc = comm.init(cfg, "escenario_combinado2")
		self.myCmdvel = self.jdrc.getCMDVelClient("escenario_combinado2.myCmdvel")
		if not self.myCmdvel:
			raise Exception("could not create client with name:myCmdvel")
		print("myCmdvel is connected")
		self.myCamera = self.jdrc.getCameraClient("escenario_combinado2.myCamera")
		if not self.myCamera:
			raise Exception("could not create client with name:myCamera")
		print("myCamera is connected")
		self.myExtra = self.jdrc.getArDroneExtraClient("escenario_combinado2.myExtra")
		if not self.myExtra:
			raise Exception("could not create client with name:myExtra")
		print("myExtra is connected")

	def destroyProxies(self):
		if self.jdrc is not None:
			self.jdrc.destroy()

	def pitchCorrection(self,t):

	

	    if (t>=0):

	

	        t = t/(math.pi) - 1

	

	    else:

	

	        t = t/(math.pi) + 1

	

	        

	

	    return t

	

	

	

	def rollCorrection(self,t):

	

	    if (t>=0):

	

	        t = (2*t)/(math.pi) - 1

	

	    else:

	

	        t = (2*t)/(math.pi) + 1

	

	        

	

	    return t

	

	

	

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

	

	

	

	def getCrosshairCoord(self, crosshair):

	

	    centerDetectedY=0

	

	    centerDetectedX=0

	

	    crossYarr=[]

	

	    crossXarr=[]

	

	    foundCenter=False

	

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

	

	                #cv2.rectangle(show_image,(posicion_x,posicion_y),(posicion_x+ancho,posicion_y+alto),(0,0,255),2)

	

	                #calculating center of crossair

	

	                crossY= (rectY+rectY+width)/2

	

	                crossX= (rectX+rectX+height)/2

	

	                #

	

	                #img = show_image+1-1

	

	                #radius = int(math.sqrt(((posicion_x+ancho)^2)+((posicion_y+alto)^2)))

	

	                #cv2.circle(show_image,(positionX,positionY), radius, (255,255,0), 1)

	

	                #cv2.imshow("Circle",show_image)

	

	                #cv2.waitKey(30)

	

	                #

	

	                crossYarr.append(crossY)

	

	                crossXarr.append(crossX)

	

	        for values in crossYarr:

	

	            centerDetectedY+=values

	

	        centerDetectedY/= len(crossYarr)

	

	        for values in crossXarr:

	

	            centerDetectedX+=values

	

	        centerDetectedX/= len(crossXarr)

	

		#

	

	    #cv2.imshow("Imagen",show_image)

	

	    #cv2.waitKey(30)

	

	    #

	

	    return centerDetectedY,centerDetectedX,foundCenter

	

	

	

	#Calculate PID

	

	def getPIDSpeed(self,error,pError,vi,cycle,kp,kd,ki):

	

	    #Proporcional

	

	    vp = kp * error

	

	    #Derivada

	

	    vd = kd * ((error-pError)/cycle)

	

	    #Integral 

	

	    viModified = ki * (vi+(error*cycle))

	

	    print "vp=%.4f" % vp,"vd=%.4f" % vd,"vi=%.4f" % viModified

	

	    #Total

	

	    result = vp + vd + viModified

	

	    return  result, viModified

	

	

	

	def getPerformance(self,start):

	

	    finish_Time = datetime.datetime.now()

	

	    dt = finish_Time - start_time

	

	    ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0

	

	    return ms #Time in miliseconds

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
		gui.addState(1, "Beacon Tracking", False, 837.0, 972.0, 0)
		gui.addState(3, "Rotational Search", False, 836.0, 777.0, 0)
		gui.addState(4, "Land", False, 1126.0, 950.0, 0)
		gui.addState(11, "Switch Beacon", False, 1047.0, 870.0, 0)
		gui.addState(12, "state 12", True, 848.0, 1147.0, 0)

		gui.addTransition(12, "Beacon Tracking time up", 1, 11, 948.5, 918.0)
		gui.addTransition(9, "Landing Time", 3, 4, 1126.5, 779.0)
		gui.addTransition(11, "Beacon Found", 3, 1, 836.0, 883.5)
		gui.addTransition(13, "Beacon Switched", 11, 3, 944.0, 822.0)

	if displayGui:
		gui.emitLoadFromRoot()
		gui.emitActiveStateById(0)

	state0 = State0(0, True, interfaces, 50, None, gui)
	state1 = State1(1, False, interfaces, 100, state0, gui)
	state3 = State3(3, False, interfaces, 100, state0, gui)
	state4 = State4(4, False, interfaces, 100, state0, gui)
	state11 = State11(11, False, interfaces, 100, state0, gui)
	state12 = State12(12, True, interfaces, 100, state0, gui)

	tran12 = Tran12(12, 11, 15000)
	state1.addTransition(tran12)

	tran9 = Tran9(9, 4, interfaces)
	state3.addTransition(tran9)

	tran11 = Tran11(11, 1, interfaces)
	state3.addTransition(tran11)

	tran13 = Tran13(13, 3, interfaces)
	state11.addTransition(tran13)

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

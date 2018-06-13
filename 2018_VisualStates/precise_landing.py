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
import sys
import traceback
import cv2
import Ice
import jderobot
import threading
import datetime
import time
import math
import xml.etree.ElementTree as ET
import numpy as np
import easyiceconfig as EasyIce


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
		start_time = datetime.datetime.now()
		
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
		hsv = cv2.cvtColor(input_image.data, cv2.COLOR_BGR2HSV)
		show_image=input_image.data+1-1
		show_image2=input_image.data+1-1
		maskOrange,maskRGBOrange = self.interfaces.hsvFilter(self.interfaces.PH_max,self.interfaces.PS_max,self.interfaces.PV_max,self.interfaces.PH_min,self.interfaces.PS_min,self.interfaces.PV_min,input_image.data,hsv)
		momentsOrange = cv2.moments(maskOrange)
		areaOrange = momentsOrange['m00']
		#print("AO=", areaOrange)
		
		maskGreen,maskRGBGreen = self.interfaces.hsvFilter(self.interfaces.SH_max,self.interfaces.SS_max,self.interfaces.SV_max,self.interfaces.SH_min,self.interfaces.SS_min,self.interfaces.SV_min,input_image.data,hsv)
		momentsGreen = cv2.moments(maskGreen)
		areaGreen = momentsGreen['m00']
		#print("AG=", areaGreen)
		
		kernel = np.ones((3,3),np.uint8)
		
		maskRGBOrange = cv2.erode(maskRGBOrange,kernel,iterations = self.interfaces.PErode)
		maskRGBOrange = cv2.dilate(maskRGBOrange,kernel,iterations = self.interfaces.PDilate)
		
		maskRGBGreen = cv2.erode(maskRGBGreen,kernel,iterations = self.interfaces.SErode)
		maskRGBGreen = cv2.dilate(maskRGBGreen,kernel,iterations = self.interfaces.SDilate)
		
		maskRGBTot = maskRGBOrange+maskRGBGreen
		
		if(self.interfaces.initialize):
		    #self.interfaces.initialTime=time.time()
		    self.interfaces.initialize=False
		    self.interfaces.centroImagen(input_image, hsv)
		    self.interfaces.myDroneExtra.takeoff()
		momentsTot = cv2.moments(maskGreen+maskOrange)
		areaTot = areaGreen + areaOrange
		# para que no divida entre cero si no empiza desde encima del coche le sumo 0.001
		xTot = int(momentsTot['m10']/(momentsTot['m00']+0.001))
		yTot = int(momentsTot['m01']/(momentsTot['m00']+0.001))
		swi=show_image+1-1
		#Try to get crossair image if close enough
		getImage,f = self.interfaces.center(show_image,maskRGBOrange,maskRGBGreen)
		positionXarr=[]
		#self.interfaces.myCMDVel.sendCMDVel(0,0,0.1,0,0,0)
		#self.interfaces.myCMDVel.sendCMDVel(0,0,0,0,0,0)
		print "f: "+str(len(f))
		#f represents the number of cuadrants detected
		if(len(f)>0):
		    #Returns Center of crossair and image with Rectangle surrounding the center of the crossair.
		    #Coordinate system is following Top-left corner. Translation of coordinates needed.
		    positionXarr,positionYarr,show_image = self.interfaces.printAndCoord(getImage,swi,f)
		    if(len(positionXarr)>0 or len(positionYarr)>0):
		        if(positionXarr[0] != -20 and positionYarr[0]!=-20):
		            #Coordinates translation to CMDVel coordinates
		            #print "X:"+str(self.interfaces.y_img)+";Y:"+str(self.interfaces.x_img)
		            #print "Xarr:"+str(positionYarr[0])+";Yarr:"+str(positionXarr[0])
		            yCenter = self.interfaces.y_img-positionYarr[0]
		            xCenter = self.interfaces.x_img-positionXarr[0]
		            print yCenter,xCenter
		            #Low-pass conservative filter (60% previoius center). Alpha = 40%
		            yError = (self.interfaces.yanterior*self.interfaces.alpha)+(yCenter*(1-self.interfaces.alpha))
		            xError = (self.interfaces.xanterior*self.interfaces.alpha)+(xCenter*(1-self.interfaces.alpha))
		            
		            self.interfaces.xanterior=xError
		            self.interfaces.yanterior=yError
		            
		            #Proportional controller
		            if(abs(yError)<25):
		                print "Vx en banda de seguridad"
		                vy=0
		                self.interfaces.yDeadZone=True
		            else:
		                vy= yError*self.interfaces.kp
		                self.interfaces.yDeadZone = False
		            if(abs(xError)<25):
		                print "Vy en banda de seguridad"
		                vx=0
		                self.interfaces.xDeadZone=True
		            else:
		                vx= xError*self.interfaces.kp
		                self.interfaces.xDeadZone=False
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
		            if abs(vx)>self.interfaces.maxSpeed:
		                print "Vx max speed reached("+str(vx)+")"
		                if vx>0:
		                    vx=self.interfaces.maxSpeed
		                else:
		                    vx=-self.interfaces.maxSpeed          
		            if abs(vy)>self.interfaces.maxSpeed:
		                print "Vy max speed reached("+str(vy)+")"
		                if vy>0:
		                    vy=self.interfaces.maxSpeed
		                else:
		                    vy=-self.interfaces.maxSpeed
		            self.interfaces.myCMDVel.sendCMDVel(vy,vx,0,0,0,0)
		            print "vx="+str(vy)+";vy="+str(vx)+";vz=0" 
		        else:
		            self.interfaces.myCMDVel.sendCMDVel(0,0,0,0,0,0)
		            print "vx=0;vy=0;vz=0.0"
		else:
		    self.interfaces.myCMDVel.sendCMDVel(0,0,0,0,0,0)
		    print "vx=0;vy=0;vz=0.0"
		            
		finish_Time = datetime.datetime.now()
		
		dt = finish_Time - start_time
		ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
		print ("response:"+ str(ms)+" ms")
		print ""
		print "******************************"
		print ""
		#Sleep not needed in Visualstates
		#if (ms < time_cycle):
		#    time.sleep((time_cycle - ms) / 1000.0)
		#End Takeoff

class State2(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		print "Landing"

class Tran1(ConditionalTransition):
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
		self.myCMDVel = None
		self.myCamera = None
		self.myDroneExtra = None
		self.myNavData = None
		self.myPose3D = None
		#self.numVuelta=50;
		#self.wSearch=0
		#self.timerW=10;
		self.xmlFileName = 'calibration.xml'
		self.initialize=True
		self.stableTime=0
		self.yanterior=0
		self.xanterior=0
		self.yanteriorTot=0
		self.xanteriorTot=0
		self.x_img=0
		self.y_img=0
		self.landed=True
		self.turnland=0
		self.numIteracionesOrange=0
		self.numIteracionesGreen=0
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
		self.xCenter = 0
		self.yCenter = 0
		self.kp = 0.003
		self.minArea = 500
		self.maxSpeed = 0.2
		self.xDeadZone = False
		self.yDeadZone = False
		self.calibration = True
		self.timeToLand = False
		self.timeLimitToLand = 10 #In seconds

		self.connectProxies()

	def connectProxies(self):
		cfg = config.load(sys.argv[1])
		self.jdrc = comm.init(cfg, "precise_landing")
		self.myCMDVel = self.jdrc.getCMDVelClient("precise_landing.myCMDVel")
		if not self.myCMDVel:
			raise Exception("could not create client with name:myCMDVel")
		print("myCMDVel is connected")
		self.myCamera = self.jdrc.getCameraClient("precise_landing.myCamera")
		if not self.myCamera:
			raise Exception("could not create client with name:myCamera")
		print("myCamera is connected")
		self.myDroneExtra = self.jdrc.getArDroneExtraClient("precise_landing.myDroneExtra")
		if not self.myDroneExtra:
			raise Exception("could not create client with name:myDroneExtra")
		print("myDroneExtra is connected")
		self.myNavData = self.jdrc.getNavdataClient("precise_landing.myNavData")
		if not self.myNavData:
			raise Exception("could not create client with name:myNavData")
		print("myNavData is connected")
		self.myPose3D = self.jdrc.getPose3dClient("precise_landing.myPose3D")
		if not self.myPose3D:
			raise Exception("could not create client with name:myPose3D")
		print("myPose3D is connected")

	def destroyProxies(self):
		if self.jdrc is not None:
			self.jdrc.destroy()

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

			

	def centroImagen(self,input_image,hsv):

	    print("CALCULA EL CENTRO DE LA IMAGEN")

	    lower_img = np.array([0,0,0], dtype=np.uint8)

	    upper_img = np.array([255,255,255], dtype=np.uint8)

	    centroimg = cv2.inRange(hsv, lower_img, upper_img)

	    momentsimg=cv2.moments(centroimg)

	    areaimg= momentsimg['m00']

	    self.x_img = int(momentsimg['m10']/momentsimg['m00'])

	    self.y_img = int(momentsimg['m01']/momentsimg['m00'])

	

	def center(self,show_image,maskRGBOrange,maskRGBGreen):

	    f = []

	    i=0

	    #calculating crossair from center

	    imgray2 = cv2.cvtColor(maskRGBOrange,cv2.COLOR_BGR2GRAY)

	    ret,thresh = cv2.threshold(imgray2,255,255,255)

	    _,contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

	    areas = [cv2.contourArea(c) for c in contours]

	    for extension in areas:

	        print "Area Primary: "+str(extension)

	        if extension > self.minArea:

	            img = np.zeros((self.y_img*2,self.x_img*2,3), np.uint8)

	            actual = contours[i]

	            approx = cv2.approxPolyDP(actual,0.05*cv2.arcLength(actual,True),True)

	            cv2.drawContours(img,[actual],0,(0,30,0),12)            

	            f.append(img)            

	            i=i+1

	    i=0

	    imgray3 = cv2.cvtColor(maskRGBGreen,cv2.COLOR_BGR2GRAY)

	    ret,thresh = cv2.threshold(imgray3,255,255,255)

	    _,contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

	    areas = [cv2.contourArea(c) for c in contours]

	    for extension in areas:

	        print "Area Secondary: "+str(extension)

	        if extension > self.minArea:

	            img = np.zeros((self.y_img*2,self.x_img*2,3), np.uint8)

	            actual = contours[i]

	            approx = cv2.approxPolyDP(actual,0.05*cv2.arcLength(actual,True),True)

	            cv2.drawContours(img,[actual],0,(0,30,0),12)

	            f.append(img)

	            i=i+1

	    

	    kernel = np.ones((5,5),np.uint8)

	    show_image2=show_image+1-1

	    if(len(f)>0):

	        f[0] = cv2.dilate(f[0],kernel,iterations = 4)

	        show_image2=f[0]

	        for k in range(len(f)-1):

	            f[k+1] = cv2.dilate(f[k+1],kernel,iterations = 4)

	            show_image2=show_image2+f[k+1]

	        

	    return show_image2,f

	

	def printAndCoord(self, show_image2,show_image,f):

	    lower_green = np.array([0,80,0], dtype=np.uint8)

	    upper_green = np.array([0, 255,0], dtype=np.uint8)

	    maskSHI = cv2.inRange(show_image2, lower_green, upper_green)

	    #filtering center from rest of the image

	    show_image2 = cv2.bitwise_and(show_image2,show_image2, mask= maskSHI)

	

	    compare_image = np.zeros((self.y_img*2,self.x_img*2,3), np.uint8)

	    diff_total = cv2.absdiff(compare_image, show_image2)

	

	    imagen_gris = cv2.cvtColor(diff_total, cv2.COLOR_BGR2GRAY)

	    _,contours,_ = cv2.findContours(imagen_gris,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

	    

	    positionXarr=[]

	    positionYarr=[]

	    positionX=-20

	    positionY=-20

	    for c in contours:

	        if(cv2.contourArea(c) >= 0):

	            posicion_x,posicion_y,ancho,alto = cv2.boundingRect(c)

	            #cv2.rectangle(show_image,(posicion_x,posicion_y),(posicion_x+ancho,posicion_y+alto),(0,0,255),2)

	            #calculating center of crossair

	            positionX= (posicion_x+posicion_x+ancho)/2

	            positionY= (posicion_y+posicion_y+alto)/2

	            #

	            #img = show_image+1-1

	            #radius = int(math.sqrt(((posicion_x+ancho)^2)+((posicion_y+alto)^2)))

	            #cv2.circle(show_image,(positionX,positionY), radius, (255,255,0), 1)

	            #cv2.imshow("Circle",show_image)

	            #cv2.waitKey(30)

	            #

	            positionXarr.append(positionX)

	            positionYarr.append(positionY)

	

	    #

	    #cv2.imshow("Imagen",show_image)

	    #cv2.waitKey(30)

	    #

	    

	    return positionXarr,positionYarr, show_image

	

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
		gui.addState(1, "Controlled Takeoff_Real", True, 862.0, 865.0, 0)
		gui.addState(2, "LandingTime", False, 1111.0, 851.0, 0)

		gui.addTransition(1, "Time to Land", 1, 2, 991.0, 917.5)

	if displayGui:
		gui.emitLoadFromRoot()
		gui.emitActiveStateById(0)

	state0 = State0(0, True, interfaces, 100, None, gui)
	state1 = State1(1, True, interfaces, 100, state0, gui)
	state2 = State2(2, False, interfaces, 100, state0, gui)

	tran1 = Tran1(1, 2, interfaces)
	state1.addTransition(tran1)

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

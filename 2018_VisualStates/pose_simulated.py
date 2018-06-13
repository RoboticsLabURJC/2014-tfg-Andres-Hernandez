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
import argparse
import mem_top


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
		
		self.interfaces.x = 0.0
		self.interfaces.y = 0.0
		self.interfaces.z = 0.0
		self.interfaces.pitch = 0.0
		for i, detection in enumerate(detections):
		    if (detection.tag_id == 4):
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
		            #ArDrone 2        [731.373, 0, 322.356, 0, 734.238, 246.304, 0, 0, 1 ]
		        cameraMatrix = np.array([187.336, 0, 160, 0, 187.336, 120, 0, 0, 1])
		        cameraMatrix = np.resize(cameraMatrix,(3,3))        
		        #Distorsion Coefficients 
		            #Gazebo         np.zeros((5, 1), np.uint8) #Comment this line when using ArDrone 2
		            #Ardrone 2      np.array([-0.073, 0.506, 0.000245, 0.000746,-1.076])
		        distCoeffs = np.zeros((5, 1), np.uint8) #Substitute Coefficients
		        retVal,rvec,tvec =cv2.solvePnP(m_MarkerPoints,detection.corners,cameraMatrix,distCoeffs)
		        rodri = cv2.Rodrigues(rvec)
		        #We get X,Y and Z
		        cameraPosition = -np.matrix(rodri[0]).T * np.matrix(tvec)
		        self.interfaces.x = cameraPosition.item(0)
		        self.interfaces.y = cameraPosition.item(1)
		        self.interfaces.z = cameraPosition.item(2)
		        #We get roll, pitch and yaw from Euler Angles        
		        eulerAngles = self.interfaces.rotationMatrixToEulerAngles(rodri[0])
		        #Conversion to Degrees
		        #yaw = eulerAngles[0]
		        self.interfaces.pitch = self.interfaces.pitchCorrection(eulerAngles.item(0))
		        #roll = rollCorrection(eulerAngles[2])
		        
		    else:  
		        print "No Beacon 4 not detected"
		    
		print ("x %.3f" % self.interfaces.x,"y %.3f" % self.interfaces.y, "z %.3f" % self.interfaces.z,"pitch %.3f" % self.interfaces.pitch)
		if (abs(self.interfaces.x)>0 and abs(self.interfaces.y)>0 and abs(self.interfaces.z)>0 and abs(self.interfaces.pitch)>0):
		    #PID sobre fuente
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

class Interfaces():
	def __init__(self):
		self.jdrc = None
		self.myCmdvel = None
		self.myCamera = None
		self.max_speed = 5
		self.dead_band_x = 0.0 #Real 0.4 #Virtual 0.2
		self.dead_band_y = 0.0 #Virtual 0.2
		self.dead_band_z = 0.0 #Real 0.4 #Virtual 0.2
		self.min_z = 2.4
		self.dead_band_w = 0.0 #0.2
		self.center = [0,0]
		self.kp = 0.2 #Real 0.05 #Virtual 0.1
		self.kd = 0.05 #Real 0.0001 #Virtual 0.03
		self.ki = 0.001 #Virtual 0.005
		self.kpz = 0.01
		self.kdz = 0.05
		self.kiz = 0.001
		self.kpw = 1
		self.kdw = 0
		self.kiw = 0
		self.cycle = 50 #in ms
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
		#Pose
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
		self.pitch = 0.0
		self.MARKER_SIZE = 0.28

		self.connectProxies()

	def connectProxies(self):
		cfg = config.load(sys.argv[1])
		self.jdrc = comm.init(cfg, "pose_simulated")
		self.myCmdvel = self.jdrc.getCMDVelClient("pose_simulated.myCmdvel")
		if not self.myCmdvel:
			raise Exception("could not create client with name:myCmdvel")
		print("myCmdvel is connected")
		self.myCamera = self.jdrc.getCameraClient("pose_simulated.myCamera")
		if not self.myCamera:
			raise Exception("could not create client with name:myCamera")
		print("myCamera is connected")

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
		gui.addState(1, "state 1", True, 869.0, 989.0, 0)


	if displayGui:
		gui.emitLoadFromRoot()
		gui.emitActiveStateById(0)

	state0 = State0(0, True, interfaces, 50, None, gui)
	state1 = State1(1, True, interfaces, 100, state0, gui)

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

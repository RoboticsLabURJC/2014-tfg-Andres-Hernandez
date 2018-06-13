self.maxSpeed = 0.3 #Real_relativa 0.07 #Cambiar max_speed
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
#self.numVuelta=50
#self.wSearch=0.0
#self.timerW=0.0
self.cycleCounter=0
#self.searchTrips=1
self.cycleFreq=(1/(self.cycle/1000.0)) #Gets how many cycles are need per second/ Our frequency
self.xSearchSpeed=0.1
self.wSearchSpeed=0.1
#self.searchSpeed=0.01
self.searchIncrement=0.05
#Initialize
self.initialize=True
self.calibration = True
#For PIDs
'''Color Beacon Variables'''
#PID Constants
self.kpxColor = 0.0001   #RealICS #Real 0.012 #Virtual 0.005
self.kdxColor = 0.05 #0.05    #Real ICS #Real  #Virtual 0.016
self.kixColor = 0.000003 #0.000001  #Real ICS #Real #Virtual 0.00004
self.kpyColor = 0.0001   #Real ICS #Real #Virtual 0.0025
self.kdyColor = 0.05 #0.05    #Real ICS #Real #Virtual 0.008
self.kiyColor = 0.000003 #0.000001  #Real ICS #Real #Virtual 0.00006
self.kpzColor = 0        
self.kdzColor = 0        
self.kizColor = 0        
self.xDistanceDeadBand = 0.1 #Virtual 0
self.yDistanceDeadBand = 0.1 #Virtual 0
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
self.timeLimitToLand = 3 #In seconds
'''Apriltag Beacon Variables'''
self.dead_band_x = 0.1 #Real_imagen #Real_pose 0.4  #Real_April 0.1 #Virtual 0.2
self.dead_band_y = 0.1 #Real_imagen #Real_pose 0.15 #Real_April 0.1 #Virtual 0.2
self.dead_band_z = 0.15 #Real_imagen #Real 0.2 #Virtual 0.2
self.min_z = 2.8       #Real 2.8
self.dead_band_w = 0.1 #Real_imagen #Real 0.180 #0.09
self.targetCenter = [0,0] #CAMBIAR
self.kp = 0.001    #Real ICS #Real_pose 0.1 #Real_April 0.01  #Virtual 0.0025
self.kd = 0       #Real ICS #Real_pose 1500 #Real_April 5 #Virtual 0.008
self.ki = 0 #Real ICS #Real_pose #Real_April 0.003 #Virtual 0.00006
self.kpw = 0.3      #Real ICS #Real 0.1 #Virtual 0.1
self.kdw = 5        #Real ICS #Real 0 #Virtual 0
self.kiw = 0        #Real ICS #Real 0 #Virtual 0
self.kpz = 0.005    #Real ICS #Real 0.006 #Virtual 0.1
self.kdz = 1        #Real ICS #Real 1 | 0.7#Virtual 0.01
self.kiz = 0.0001  #Real ICS # Real 0.00006# Virtual 0.001
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

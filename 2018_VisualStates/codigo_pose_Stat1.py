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
            retVal,rvec,tvec =cv2.solvePnP(self.interfaces.m_MarkerPoints
                                        ,detection.corners
                                        ,self.interfaces.cameraMatrix
                                        ,self.interfaces.distCoeffs)
            rodri = cv2.Rodrigues(rvec)
            #We get X,Y and Z
            cameraPosition = -np.matrix(rodri[0]).T * np.matrix(tvec) 
            self.interfaces.x = detection.center[0]
            self.interfaces.y = detection.center[1]
            #xApril = detection.center[0]
            #yApril = detection.center[1]
            #self.interfaces.x = cameraPosition.item(0) #Center needs to be changed
            #self.interfaces.y = cameraPosition.item(1) #Center needs to be changed
            self.interfaces.z = cameraPosition.item(2)
            #We get roll, pitch and yaw from Euler Angles        
            eulerAngles = self.interfaces.rotationMatrixToEulerAngles(rodri[0])
            #yaw = eulerAngles[0]
            #pitch = eulerAngles[1]
            #roll = self.interfaces.rollCorrection(eulerAngles[2])
            #Threshold for pitch correction (center of the width of the image)
            th=inputImageData_w/2
            self.interfaces.pitch = self.interfaces.pitchCorrection(self.interfaces.x,th,eulerAngles.item(1))
            #self.interfaces.pitch = self.interfaces.pitchCorrection(xApril,th,eulerAngles.item(1)) #POSE
            #print xApril, yApril, self.interfaces.pitch
        
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
#logging.info ("2-vz:%.3f;" % -vz + "vx:%.3f;" % -vx+  "vy:%.3f;" % vy +"vw:%.3f" % vw) #Pose
logging.info ("2-vz:%.3f;" % -vz + "vy:%.3f;" % vy + "vx:%.3f;" % vx +"vw:%.3f" % vw) #AprilTags
#print ("2-vz:%.3f;" % -vz + "vx:%.3f;" % -vx+  "vy:%.3f;" % vy +"vw:%.3f" % vw)        #Pose
print ("2-vz:%.3f;" % -vz + "vx:%.3f;" % vx + "vy:%.3f;" % vy +"vw:%.3f" % vw)      #AprilTags
#Send Speeds
#self.interfaces.myCmdvel.sendCMDVel(-vz,-vx,vy,0,0,vw) #Center from pose
self.interfaces.myCmdvel.sendCMDVel(-vz, vx,vy,0,0,vw)  #Center from Apriltags
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

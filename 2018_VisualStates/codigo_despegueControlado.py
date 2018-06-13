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
            print yError, xError 
            #Y Controller
            if(abs(yError)<self.interfaces.yDistanceDeadBand):
                print "Vy en banda de seguridad"
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
                print "vx en banda de seguridad"
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
    print yError, xError
    #Y Controller
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
    #Get vx Speed thanks to the PID controller and checks if in Dead band.
    vx,vxiMod = self.interfaces.getPIDSpeed(xError
                                    ,self.interfaces.errXPrevColor
                                    ,self.interfaces.vxi
                                    ,self.interfaces.cycle
                                    ,self.interfaces.kpxColor
                                    ,self.interfaces.kdxColor
                                    ,self.interfaces.kixColor)
    self.interfaces.vxi=vxiMod
elif(areaPrimary==0 and areaSecondary>0):
    print "Only Secondary"
    ySecondary = momentsSecondary['m10']/momentsSecondary['m00']
    xSecondary = momentsSecondary['m01']/momentsSecondary['m00']
    yError = self.interfaces.center[1]-ySecondary
    xError = self.interfaces.center[0]-xSecondary
    print yError, xError
    #Y Controller
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

#End Takeoff

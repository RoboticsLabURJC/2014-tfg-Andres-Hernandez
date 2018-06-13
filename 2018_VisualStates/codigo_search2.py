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
    self.interfaces.myCmdvel.sendCMDVel(-self.interfaces.xSearchSpeed,0,0,0,0,-self.interfaces.wSearchSpeed)
else:
    numVuelta=numVuelta+1
    if(numVuelta>100 and numVuelta < 120):
        self.cmdvel.sendCMDVel(1.8+wSearch,0,0,0,0,-1.5)
        #timerW=timerW+(timerW/8)
        if(numVuelta==119):
            numVuelta=0
        if(wSearch<1 and numVuelta==101):
           wSearch=wSearch+0.2
    else:
        self.cmdvel.sendCMDVel(1.8+wSearch,0,0,0,0,1.5 - wSearch)

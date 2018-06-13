start_time = datetime.datetime.now()

if(self.interfaces.initTime==0):
    tree = ET.parse('calibration.xml')
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
    print "calibration from file calibration_simulated.xml completed"

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

if(self.interfaces.initTime==0):
    self.interfaces.initialTime=time.time()
    self.interfaces.initTime=1
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
self.interfaces.myCMDVel.sendCMDVel(0,0,0,0,0,0)
print "f: "+str(len(f))
#f represents the number of cuadrants detected
if(len(f)>0):
    #Returns Center of crossair and image with Rectangle surrounding the center of the crossair.
    #Coordinate system is following Top-left corner. Translation of coordinates needed.
    positionXarr,positionYarr,show_image = self.interfaces.printAndCoord(getImage,swi,f)
    if(len(positionXarr)>0):
        if(positionXarr[0] != -20 and positionYarr[0]!=-20):
            #Coordinates translation to CMDVel coordinates
            print "X:"+str(self.interfaces.y_img)+";Y:"+str(self.interfaces.x_img)
            print "Xarr:"+str(positionYarr[0])+";Yarr:"+str(positionXarr[0])
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
            else:
                vy= yError*self.kp #0.01
                
            if(abs(xError)<25):
                print "Vy en banda de seguridad"
                vx=0
            else:
                vx= xError*self.kp #0.01
# cambio de la velocidad inicial para que no se vaya tan lejos
            self.interfaces.myCMDVel.sendCMDVel(vy,vx,0,0,0,0)
            print "vx="+str(vy)+";vy="+str(vx)+";vz=0"
            
        else:
            #self.interfaces.myCMDVel.sendCMDVel(0,0,0.1,0,0,0)
            self.interfaces.myCMDVel.sendCMDVel(0,0,0,0,0,0)
            print "vx=0;vy=0;vz=0.1"
else:
    #self.interfaces.myCMDVel.sendCMDVel(0,0,0.1,0,0,0)
    self.interfaces.myCMDVel.sendCMDVel(0,0,0,0,0,0)
    print "vx=0;vy=0;vz=0.1"
            
finish_Time = datetime.datetime.now()

dt = finish_Time - start_time
ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
print ("response:"+ str(ms)+" ms")

#Sleep not needed in Visualstates
#if (ms < time_cycle):
#    time.sleep((time_cycle - ms) / 1000.0)
#End Takeoff

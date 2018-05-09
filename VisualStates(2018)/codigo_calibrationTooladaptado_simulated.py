#self.interfaces.myDroneExtra.takeoff()

start_time = datetime.datetime.now()

if(self.interfaces.initTime==0):
    tree = ET.parse('calibration_simulated.xml')
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

#while(input_image is not None and self.interfaces.turntake == 0):
input_image = self.interfaces.myCamera.getImage()
hsv = cv2.cvtColor(input_image.data, cv2.COLOR_BGR2HSV)
show_image=input_image.data+1-1
show_image2=input_image.data+1-1
maskOrange,maskRGBOrange = self.interfaces.hsvFilter(self.interfaces.PH_max,self.interfaces.PS_max,self.interfaces.PV_max,self.interfaces.PH_min,self.interfaces.PS_min,self.interfaces.PV_min,input_image.data,hsv)
#lower_orange = np.array([self.interfaces.PH_min,self.interfaces.PS_min,self.interfaces.PV_min], dtype=np.uint8)
#upper_orange = np.array([self.interfaces.PH_max,self.interfaces.PS_max,self.interfaces.PV_max], dtype=np.uint8)
#maskOrange = cv2.inRange(hsv, lower_orange, upper_orange)
#maskRGBOrange = cv2.bitwise_and(input_image.data,input_image.data, mask= maskOrange)

momentsOrange = cv2.moments(maskOrange)
areaOrange = momentsOrange['m00']
#print("AO=", areaOrange)

maskGreen,maskRGBGreen = self.interfaces.hsvFilter(self.interfaces.SH_max,self.interfaces.SS_max,self.interfaces.SV_max,self.interfaces.SH_min,self.interfaces.SS_min,self.interfaces.SV_min,input_image.data,hsv)
#lower_green = np.array([self.interfaces.SH_min,self.interfaces.SS_min,self.interfaces.SV_min], dtype=np.uint8)
#upper_green = np.array([self.interfaces.SH_max,self.interfaces.SS_max,self.interfaces.SV_max], dtype=np.uint8)
#maskGreen = cv2.inRange(hsv, lower_green, upper_green)
#maskRGBGreen = cv2.bitwise_and(input_image.data,input_image.data, mask= maskGreen)

momentsGreen = cv2.moments(maskGreen)
areaGreen = momentsGreen['m00']
#print("AG=", areaGreen)

kernel = np.ones((3,3),np.uint8)

maskRGBOrange = cv2.erode(maskRGBOrange,kernel,iterations = self.interfaces.PErode)
maskRGBOrange = cv2.dilate(maskRGBOrange,kernel,iterations = self.interfaces.PDilate)

maskRGBGreen = cv2.erode(maskRGBGreen,kernel,iterations = self.interfaces.SErode)
maskRGBGreen = cv2.dilate(maskRGBGreen,kernel,iterations = self.interfaces.SDilate)

kp=0.01
kd=0.003

maskRGBTot = maskRGBOrange+maskRGBGreen

if(-self.interfaces.initialTime+time.time()<10 or self.interfaces.initTime==0):
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
        #Returns Center of crossair and image with Rectangle surrounding the center of the crossair
        positionXarr,positionYarr,show_image = self.interfaces.printAndCoord(getImage,swi,f)
        if(len(positionXarr)>0):
            if(positionXarr[0] != -20 and positionYarr[0]!=-20):
                vely = (self.interfaces.y_img-positionYarr[0])
                velx = (self.interfaces.x_img-positionXarr[0])

                vytot= vely*kp #0.01
                vxtot= velx*kp #0.01

                velxa=1-abs(self.interfaces.xanterior-velx)/10 #10
                if(velxa<0.1):
                    velxa=0.1
                
                velya=1-abs(self.interfaces.yanterior-vely)/10 #10
                if(velya<0.1):
                    velya=0.1

                if(abs(self.interfaces.y_img-positionYarr[0])<25):
                    print "Vx en banda de seguridad"
                    vy=0
                else:
                    #vy=vytot*velya*1.4
                    vy=(vytot*0.6)+(velya*0.4)

                if(abs(self.interfaces.x_img-positionXarr[0])<25):
                    print "Vy en banda de seguridad"
                    vx=0
                else:
                    #vx=vxtot*velxa*1.4
                    vx=(vxtot*0.6)+(velxa*0.4)
    # cambio de la velocidad inicial para que no se vaya tan lejos
                self.interfaces.myCMDVel.sendCMDVel(vy*0.5,vx*0.5,0,0,0,0)
                print "vx="+str(vy*0.5)+";vy="+str(vx*0.5)+";vz=0"
                self.interfaces.yanterior=vx
                self.interfaces.xanterior=vy
            else:
                #self.interfaces.myCMDVel.sendCMDVel(0,0,0.1,0,0,0)
                self.interfaces.myCMDVel.sendCMDVel(0,0,0,0,0,0)
                print "vx=0;vy=0;vz=0.1"
    else:
        #self.interfaces.myCMDVel.sendCMDVel(0,0,0.1,0,0,0)
        self.interfaces.myCMDVel.sendCMDVel(0,0,0,0,0,0)
        print "vx=0;vy=0;vz=0.1"

else:

    if(areaOrange > 0 and areaGreen == 0):
        self.interfaces.numIteracionesOrange=self.interfaces.numIteracionesOrange+1
        print("orange")
        xOrange = int(momentsOrange['m10']/momentsOrange['m00'])
        yOrange = int(momentsOrange['m01']/momentsOrange['m00'])

        vely = (self.interfaces.y_img-yOrange)
        velx = (self.interfaces.x_img-xOrange)

        vytot= vely*kp
        vxtot= velx*kp

        velxa=abs(self.interfaces.xanterior-velx)*kd
        velya=abs(self.interfaces.yanterior-vely)*kd

        vytot=(vytot+velya)
        vxtot=(vxtot+velxa)

        if(abs(vxtot-self.interfaces.xanteriorTot)>0.3):
            if(vxtot<self.interfaces.xanteriorTot):
                vxtot = self.interfaces.xanteriorTot-0.3
            else:
                vxtot = self.interfaces.xanteriorTot+0.3

            if(abs(vytot-self.interfaces.yanteriorTot)>0.3):
                if(vytot<self.interfaces.yanteriorTot):
                    vytot = self.interfaces.yanteriorTot-0.3
                else:
                    vytot = self.interfaces.yanteriorTot+0.3
                self.interfaces.yanteriorTot=vytot
                self.interfaces.xanteriorTot=vxtot
                self.interfaces.myCMDVel.sendCMDVel(vytot*0.5,vxtot*0.5,0,0,0,0)
                print "vx="+str(vytot*0.5)+";vy="+str(vxtot*0.5)+";vz=0"

    elif(areaOrange > 0 and areaGreen>0):
        momentsTot = cv2.moments(maskGreen+maskOrange)
        areaTot = areaGreen + areaOrange
        xTot = int(momentsTot['m10']/momentsTot['m00'])
        yTot = int(momentsTot['m01']/momentsTot['m00'])
        print("green and orange")

        if((abs(self.interfaces.y_img-yTot)<=6 and abs(self.interfaces.x_img-xTot)<=6)):
            if(self.interfaces.turntake==0):
                if(areaTot<9272135.0):
                    #self.interfaces.myDroneExtra.takeoff()
                    self.interfaces.turntake=1
                    self.interfaces.landed=time.time()
                else:
                    #self.interfaces.myCMDVel.sendCMDVel(0,0,0.1,0,0,0)
                    self.interfaces.myCMDVel.sendCMDVel(0,0,0,0,0,0)
                    print "vx=0;vy=0;vz=0.1"
                    print(areaTot)
        elif(self.interfaces.landed==0):
            kernel = np.ones((3,3),np.uint8)
            maskRGBTot = cv2.erode(maskRGBTot,kernel,iterations =2)
            maskRGBTot = cv2.dilate(maskRGBTot,kernel,iterations =2)

            vely = (self.interfaces.y_img-yTot)
            velx = (self.interfaces.x_img-xTot)

            vytot= vely*kp
            vxtot= velx*kp

            velxa=1-abs(self.interfaces.xanterior-velx)/10
            if(velxa<0.1):
                velxa=0.1

            velya=1-abs(self.interfaces.yanterior-vely)/10
            if(velya<0.1):
                velya=0.1

            self.interfaces.yanterior = self.interfaces.y_img-yTot
            self.interfaces.xanterior = self.interfaces.x_img-xTot

            swi=show_image+1-1
            getImage,f = self.interfaces.center(show_image,maskRGBOrange,maskRGBGreen)
            show_image4=getImage
            positionXarr=[]
            if(len(f) >0):
                positionXarr,positionYarr,show_image = self.interfaces.printAndCoord(getImage,swi,f)

            blank_image = np.zeros((self.interfaces.y_img*2,self.interfaces.x_img*2,3), np.uint8)

            positionX = -20
            positionY = -20
            if(len(positionXarr)>0):
                positionX=positionXarr[0]
                positionY=positionYarr[0]

                if(positionX != 0 ):
                    vely = (self.interfaces.y_img-positionYarr[0])
                    velx = (self.interfaces.x_img-positionXarr[0])

                    vytot= vely*kp
                    vxtot= velx*kp

                    velxa=abs(self.interfaces.xanterior-velx)*kd
                    velya=abs(self.interfaces.yanterior-vely)*kd

                    if(abs(vxtot-self.interfaces.xanteriorTot)>0.3):
                        if(vxtot<self.interfaces.xanteriorTot):
                            vxtot = self.interfaces.xanteriorTot-0.3
                        else:
                            vxtot = self.interfaces.xanteriorTot+0.3

                    if(abs(vytot-self.interfaces.yanteriorTot)>0.3):
                        if(vytot<self.interfaces.yanteriorTot):
                            vytot = self.interfaces.yanteriorTot-0.3
                        else:
                            vytot = self.interfaces.yanteriorTot+0.3
                    self.interfaces.yanterior=velya
                    self.interfaces.xanterior=velxa
                    self.interfaces.myCMDVel.sendCMDVel(vytot*0.5,vxtot*0.5,0,0,0,0)
                    print "vx="+str(vytot*0.5)+";vy="+str(vxtot*0.5)+";vz=0"
                    self.interfaces.yanteriorTot=vytot
                    self.interfaces.xanteriorTot=vxtot
                else:
                    if(abs(vxtot-self.interfaces.xanteriorTot)>0.3):
                        if(vxtot<self.interfaces.xanteriorTot):
                            vxtot = self.interfaces.xanteriorTot-0.3
                        else:
                            vxtot = self.interfaces.xanteriorTot+0.3

                    if(abs(vytot-self.interfaces.yanteriorTot)>0.3):
                        if(vytot<self.interfaces.yanteriorTot):
                            vytot = self.interfaces.yanteriorTot-0.3
                        else:
                            vytot = self.interfaces.yanteriorTot+0.3
                    self.interfaces.yanterior=velya
                    self.interfaces.xanterior=velxa
                    self.interfaces.myCMDVel.sendCMDVel(vytot*0.5,vxtot*0.5,0,0,0,0)
                    print "vx="+str(vytot*0.5)+";vy="+str(vxtot*0.5)+";vz=0"
                    self.interfaces.yanteriorTot=vytot
                    self.interfaces.xanteriorTot=vxtot

            else:
                velxa=abs(self.interfaces.xanterior-velx)*kd
                velya=abs(self.interfaces.yanterior-vely)*kd
                #print(vytot+velya)
                vytot=(vytot+velya)
                vxtot=(vxtot+velxa)

                if(abs(vxtot-self.interfaces.xanteriorTot)>0.3):
                    if(vxtot<self.interfaces.xanteriorTot):
                        vxtot = self.interfaces.xanteriorTot-0.3
                    else:
                        vxtot = self.interfaces.xanteriorTot+0.3

                if(abs(vytot-self.interfaces.yanteriorTot)>0.3):
                    if(vytot<self.interfaces.yanteriorTot):
                        vytot = self.interfaces.yanteriorTot-0.3
                    else:
                        vytot = self.interfaces.yanteriorTot+0.3
                self.interfaces.yanteriorTot=vytot
                self.interfaces.xanteriorTot=vxtot
                self.interfaces.myCMDVel.sendCMDVel(vytot*0.5,vxtot*0.5,0,0,0,0)
                print "vx="+str(vytot*0.5)+";vy="+str(vxtot*0.5)+";vz=0"
    elif(areaOrange == 0 and areaGreen>0):
        self.interfaces.numIteracionesGreen=self.interfaces.numIteracionesGreen+1
        self.interfaces.var_beacon_status = 1
        print("green")

        xGreen = int(momentsGreen['m10']/momentsGreen['m00'])
        yGreen = int(momentsGreen['m01']/momentsGreen['m00'])
        if(self.interfaces.yanterior==0 and self.interfaces.xanterior==0):
            self.interfaces.yanterior = (self.interfaces.y_img-yGreen)*0.02
            self.interfaces.xanterior = (self.interfaces.x_img-xGreen)*0.02
            self.interfaces.myCMDVel.sendCMDVel(self.interfaces.yanterior*0.5,self.interfaces.xanterior*0.5,0,0,0,0)
            print "vx="+str(self.interfaces.yanterior*0.5)+";vy="+str(self.interfaces.xanterior*0.5)+";vz=0"
            vely = self.interfaces.yanterior
            velx = self.interfaces.xanterior
        else:
            vely = (self.interfaces.y_img-yGreen)
            velx = (self.interfaces.x_img-xGreen)

        vytot= vely*kp
        vxtot= velx*kp

        velxa=abs(self.interfaces.xanterior-velx)*kd
        velya=abs(self.interfaces.yanterior-vely)*kd

        vytot=(vytot+velya)
        vxtot=(vxtot+velxa)

        if(abs(vxtot-self.interfaces.xanteriorTot)>0.3):
            if(vxtot<self.interfaces.xanteriorTot):
                vxtot = self.interfaces.xanteriorTot-0.3
            else:
                vxtot = self.interfaces.xanteriorTot+0.3

            if(abs(vytot-self.interfaces.yanteriorTot)>0.3):
                if(vytot<self.interfaces.yanteriorTot):
                    vytot = self.interfaces.yanteriorTot-0.3
                else:
                    vytot = self.interfaces.yanteriorTot+0.3
                self.interfaces.yanteriorTot=vytot
                self.interfaces.xanteriorTot=vxtot
                self.interfaces.myCMDVel.sendCMDVel(vytot*0.5,vxtot*0.5,0,0,0,0)
                print "vx="+str(self.interfaces.yanterior*0.5)+";vy="+str(self.interfaces.xanterior*0.5)+";vz=0"
    else:
        self.interfaces.turntake = 1
            
finish_Time = datetime.datetime.now()

dt = finish_Time - start_time
ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
print ("response:"+ str(ms)+" ms")

#Sleep not needed in Visualstates
#if (ms < time_cycle):
#    time.sleep((time_cycle - ms) / 1000.0)
#End Takeoff

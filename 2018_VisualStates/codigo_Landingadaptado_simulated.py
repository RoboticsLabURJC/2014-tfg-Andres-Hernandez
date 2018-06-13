#self.interfaces.myDroneExtra.toggleCam()

start_time = datetime.datetime.now()

input_image = self.interfaces.myCamera.getImage()

hsv = cv2.cvtColor(input_image.data, cv2.COLOR_BGR2HSV)
show_image=input_image.data+1-1
show_image2=input_image.data+1-1
maskOrange,maskRGBOrange = self.interfaces.hsvFilter(self.interfaces.PH_max,self.interfaces.PS_max,self.interfaces.PV_max,self.interfaces.PH_min,self.interfaces.PS_min,self.interfaces.PV_min,input_image.data,hsv)

momentsOrange = cv2.moments(maskOrange)
areaOrange = momentsOrange['m00']

maskGreen,maskRGBGreen = self.interfaces.hsvFilter(self.interfaces.SH_max,self.interfaces.SS_max,self.interfaces.SV_max,self.interfaces.SH_min,self.interfaces.SS_min,self.interfaces.SV_min,input_image.data,hsv)

momentsGreen = cv2.moments(maskGreen)
areaGreen = momentsGreen['m00']

kernel = np.ones((3,3),np.uint8)

maskRGBOrange = cv2.erode(maskRGBOrange,kernel,iterations = self.interfaces.PErode)
maskRGBOrange = cv2.dilate(maskRGBOrange,kernel,iterations = self.interfaces.PDilate)

maskRGBGreen = cv2.erode(maskRGBGreen,kernel,iterations = self.interfaces.SErode)
maskRGBGreen = cv2.dilate(maskRGBGreen,kernel,iterations = self.interfaces.SDilate)

maskRGBTot = maskRGBOrange+maskRGBGreen


if(areaOrange > 0 and areaGreen == 0):
    #numIteracionesOrange=numIteracionesOrange+1
    print("orange")
    xOrange = int(momentsOrange['m10']/momentsOrange['m00'])
    yOrange = int(momentsOrange['m01']/momentsOrange['m00'])

    if(self.interfaces.yanterior==0 and self.interfaces.xanterior==0):
        self.interfaces.yanterior = (self.interfaces.y_img-xOrange)*0.02
        self.interfaces.xanterior = (self.interfaces.x_img-yOrange)*0.02
        #self.interfaces.myCMDVel.sendCMDVel(yanterior*0.5,self.interfaces.xanterior*0.5,0,0,0,0)
        vely = self.interfaces.yanterior
        velx = self.interfaces.xanterior
    vely = (self.interfaces.y_img-yOrange)
    velx = (self.interfaces.x_img-xOrange)

    vytot= vely*self.interfaces.kp
    vxtot= velx*self.interfaces.kp

    velxa=abs(self.interfaces.xanterior-velx)*self.interfaces.kd
    velya=abs(self.interfaces.yanterior-vely)*self.interfaces.kd

    self.interfaces.yanterior = (self.interfaces.y_img-xOrange)*0.02
    self.interfaces.xanterior = (self.interfaces.x_img-yOrange)*0.02

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

elif(areaOrange > 0 and areaGreen>0):
    momentsTot = cv2.moments(maskGreen+maskOrange)
    areaTot = areaGreen + areaOrange
    xTot = int(momentsTot['m10']/momentsTot['m00'])
    yTot = int(momentsTot['m01']/momentsTot['m00'])
    print("green and orange")

    if((abs(self.interfaces.y_img-yTot)<=6 and abs(self.interfaces.x_img-xTot)<=6)):
        if(self.interfaces.turnland==0):
            if(areaTot>10272135.0):
                self.interfaces.myDroneExtra.land()
                self.interfaces.turnland=1
                self.interfaces.landed=time.time()
            else:
                self.interfaces.myCMDVel.sendCMDVel(0,0,-0.3,0,0,0)
                print(areaTot)
    elif(self.interfaces.landed==0):
        kernel = np.ones((3,3),np.uint8)
        maskRGBTot = cv2.erode(maskRGBTot,kernel,iterations =2)
        maskRGBTot = cv2.dilate(maskRGBTot,kernel,iterations =2)

        vely = (self.interfaces.y_img-yTot)
        velx = (self.interfaces.x_img-xTot)

        vytot= vely*self.interfaces.kp
        vxtot= velx*self.interfaces.kp

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

                vytot= vely*self.interfaces.kp
                vxtot= velx*self.interfaces.kp

                velxa=abs(self.interfaces.xanterior-velx)*self.interfaces.kd
                velya=abs(self.interfaces.yanterior-vely)*self.interfaces.kd

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
                self.interfaces.yanteriorTot=vytot
                self.interfaces.xanteriorTot=vxtot

        else:
            velxa=abs(self.interfaces.xanterior-velx)*self.interfaces.kd
            velya=abs(self.interfaces.yanterior-vely)*self.interfaces.kd
            print(vytot+velya)
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
elif(areaOrange == 0 and areaGreen>0):
    #numIteracionesGreen=numIteracionesGreen+1
    #var_beacon_status = 1
    print("green")

    xGreen = int(momentsGreen['m10']/momentsGreen['m00'])
    yGreen = int(momentsGreen['m01']/momentsGreen['m00'])
    if(self.interfaces.yanterior==0 and self.interfaces.xanterior==0):
        self.interfaces.yanterior = (self.interfaces.y_img-yGreen)*0.02
        self.interfaces.xanterior = (self.interfaces.x_img-xGreen)*0.02
        #self.interfaces.myCMDVel.sendCMDVel(yanterior*0.5,self.interfaces.xanterior*0.5,0,0,0,0)
        vely = self.interfaces.yanterior
        velx = self.interfaces.xanterior
    else:
        vely = (self.interfaces.y_img-yGreen)
        velx = (self.interfaces.x_img-xGreen)

    vytot= vely*self.interfaces.kp
    vxtot= velx*self.interfaces.kp

    velxa=abs(self.interfaces.xanterior-velx)*self.interfaces.kd
    velya=abs(self.interfaces.yanterior-vely)*self.interfaces.kd
    
    self.interfaces.yanterior = (self.interfaces.y_img-yGreen)*0.02
    self.interfaces.xanterior = (self.interfaces.x_img-xGreen)*0.02

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


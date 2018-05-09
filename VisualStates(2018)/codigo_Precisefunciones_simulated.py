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
    m=input_image.data+1-1
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
            #cv2.circle(img,(positionX,positionY), radius, (255,255,0), 1)
            #cv2.imshow("Circle",img)
            #cv2.waitKey(30)
            #
            positionXarr.append(positionX)
            positionYarr.append(positionY)

    #
    #cv2.imshow("Imagen",show_image)
    #cv2.waitKey(30)
    #
    
    return positionXarr,positionYarr, show_image


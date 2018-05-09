import sys, traceback, Ice
import easyiceconfig as EasyIce
import jderobot
import numpy as np
import threading
import xml.etree.ElementTree as ET
import cv2

def nothing(x):
	pass

def exit():
    exit
    
def hsvFilter(hup,sup,vup,hdwn,sdwn,vdwn,img,hsv):
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

#Init Calibration Windows
cv2.namedWindow('filtered_image')
cv2.namedWindow('Primary_mask')
cv2.namedWindow('Secondary_mask')

#Init trackBars
cv2.createTrackbar('PH_max','filtered_image',0,180,nothing)
cv2.createTrackbar('PS_max','filtered_image',0,255,nothing)
cv2.createTrackbar('PV_max','filtered_image',0,255,nothing)
cv2.createTrackbar('PH_min','filtered_image',0,255,nothing)
cv2.createTrackbar('PS_min','filtered_image',0,255,nothing)
cv2.createTrackbar('PV_min','filtered_image',0,255,nothing)
cv2.createTrackbar('PErode','filtered_image',0,100,nothing)
cv2.createTrackbar('PDilate','filtered_image',0,100,nothing)

cv2.createTrackbar('SH_max','filtered_image',0,180,nothing)
cv2.createTrackbar('SS_max','filtered_image',0,255,nothing)
cv2.createTrackbar('SV_max','filtered_image',0,255,nothing)
cv2.createTrackbar('SH_min','filtered_image',0,255,nothing)
cv2.createTrackbar('SS_min','filtered_image',0,255,nothing)
cv2.createTrackbar('SV_min','filtered_image',0,255,nothing)
cv2.createTrackbar('SErode','filtered_image',0,100,nothing)
cv2.createTrackbar('SDilate','filtered_image',0,100,nothing)

if __name__ == "__main__":
    if(len(sys.argv)==3):
        config = sys.argv[2]
    else:
        cv2.destroyAllWindows()
        sys.exit("Usage: python2 calibrationTool.py iceConfig.cfg xmlConfig.xml")
    ic = EasyIce.initialize(sys.argv)
    properties = ic.getProperties()
    basecameraL = ic.propertyToProxy("Camera.Proxy")
    cameraProxy = jderobot.CameraPrx.checkedCast(basecameraL)        
    #xml values
    tree = ET.parse(config)
    root = tree.getroot()
    for colour in root.findall('colour'):
        name = colour.get('name')
        if (name=="Primary"):
            PH_max = int(colour.find('Hmax').text)
            PS_max = int(colour.find('Smax').text)
            PV_max = int(colour.find('Vmax').text)
            PH_min = int(colour.find('Hmin').text)
            PS_min = int(colour.find('Smin').text)
            PV_min = int(colour.find('Vmin').text)
            PErode = int(colour.find('Erosion').text)
            PDilate = int(colour.find('Dilation').text)
            
        elif(name=="Secondary"):
            SH_max = int(colour.find('Hmax').text)
            SS_max = int(colour.find('Smax').text)
            SV_max = int(colour.find('Vmax').text)
            SH_min = int(colour.find('Hmin').text)
            SS_min = int(colour.find('Smin').text)
            SV_min = int(colour.find('Vmin').text)
            SErode = int(colour.find('Erosion').text)
            SDilate = int(colour.find('Dilation').text)
            
          
    #init
    cv2.setTrackbarPos('PH_max','filtered_image',PH_max)
    cv2.setTrackbarPos('PS_max','filtered_image',PS_max)
    cv2.setTrackbarPos('PV_max','filtered_image',PV_max)
    cv2.setTrackbarPos('PH_min','filtered_image',PH_min)
    cv2.setTrackbarPos('PS_min','filtered_image',PS_min)
    cv2.setTrackbarPos('PV_min','filtered_image',PV_min)
    cv2.setTrackbarPos('PErode','filtered_image',PErode)
    cv2.setTrackbarPos('PDilate','filtered_image',PDilate)
    
    cv2.setTrackbarPos('SH_max','filtered_image',SH_max)
    cv2.setTrackbarPos('SS_max','filtered_image',SS_max)
    cv2.setTrackbarPos('SV_max','filtered_image',SV_max)
    cv2.setTrackbarPos('SH_min','filtered_image',SH_min)
    cv2.setTrackbarPos('SS_min','filtered_image',SS_min)
    cv2.setTrackbarPos('SV_min','filtered_image',SV_min)
    cv2.setTrackbarPos('SErode','filtered_image',SErode)
    cv2.setTrackbarPos('SDilate','filtered_image',SDilate)

    #Morphological transforms vars
    kernel = np.ones((3,3),np.uint8)
    
    key=-1
    while key != 1048689:
        imageData = cameraProxy.getImageData("RGB8")
        imageData_h = imageData.description.height
        imageData_w = imageData.description.width
        image = np.zeros((imageData_h, imageData_w, 3), np.uint8)
        image = np.frombuffer(imageData.pixelData, dtype=np.uint8)
        image.shape = imageData_h, imageData_w, 3
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        cv2.imshow("filtered_image", image)
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        #Green
        PH_min = cv2.getTrackbarPos('PH_min','filtered_image')
        PS_min = cv2.getTrackbarPos('PS_min','filtered_image')
        PV_min = cv2.getTrackbarPos('PV_min','filtered_image')
        PH_max = cv2.getTrackbarPos('PH_max','filtered_image')
        PS_max = cv2.getTrackbarPos('PS_max','filtered_image')
        PV_max = cv2.getTrackbarPos('PV_max','filtered_image')
        PErode = cv2.getTrackbarPos('PErode','filtered_image')
        PDilate = cv2.getTrackbarPos('PDilate','filtered_image')
        lower_primary = np.array([PH_min,PS_min,PV_min], dtype=np.uint8)
        upper_primary = np.array([PH_max,PS_max,PV_max], dtype=np.uint8)
        #Mask
        Pmask, PmaskRGB = hsvFilter(PH_max,PS_max,PV_max,PH_min,PS_min,PV_min,image,hsv)
        #Pmask = cv2.inRange(hsv, lower_primary, upper_primary)
        #PmaskRGB = cv2.bitwise_and(image,image, mask= Pmask)
        #Erosion/Dilation
        PmaskRGB = cv2.erode(PmaskRGB,kernel,iterations = PErode)
        PmaskRGB = cv2.dilate(PmaskRGB,kernel,iterations = PDilate)
        
        #Orange/Blue
        SH_min = cv2.getTrackbarPos('SH_min','filtered_image')
        SS_min = cv2.getTrackbarPos('SS_min','filtered_image')
        SV_min = cv2.getTrackbarPos('SV_min','filtered_image')
        SH_max = cv2.getTrackbarPos('SH_max','filtered_image')
        SS_max = cv2.getTrackbarPos('SS_max','filtered_image')
        SV_max = cv2.getTrackbarPos('SV_max','filtered_image')
        SErode = cv2.getTrackbarPos('SErode','filtered_image')
        SDilate = cv2.getTrackbarPos('SDilate','filtered_image')
        lower_secondary = np.array([SH_min,SS_min,SV_min], dtype=np.uint8)
        upper_secondary = np.array([SH_max,SS_max,SV_max], dtype=np.uint8)
        #Mask
        Smask = cv2.inRange(hsv, lower_secondary, upper_secondary)
        SmaskRGB = cv2.bitwise_and(image,image, mask= Smask)
        #Erosion/Dilation
        SmaskRGB = cv2.erode(SmaskRGB,kernel,iterations = SErode)
        SmaskRGB = cv2.dilate(SmaskRGB,kernel,iterations = SDilate)
        
        #show results
        filterImage = PmaskRGB+SmaskRGB
        cv2.imshow("filtered_image", filterImage)
        cv2.imshow("Primary_mask", PmaskRGB)
        cv2.imshow("Secondary_mask", SmaskRGB)
        key=cv2.waitKey(30)
        if (key==27):
            for colour in root.findall('colour'):
                name = colour.get('name')
                if (name=="Primary"):
                    colour.find('Hmax').text = str(PH_max)
                    colour.find('Smax').text = str(PS_max)
                    colour.find('Vmax').text = str(PV_max)
                    colour.find('Hmin').text = str(PH_min)
                    colour.find('Smin').text = str(PS_min)
                    colour.find('Vmin').text = str(PV_min)
                    colour.find('Erosion').text = str(PErode)
                    colour.find('Dilation').text = str(PDilate)
                    
                elif(name=="Secondary"):
                    colour.find('Hmax').text = str(SH_max)
                    colour.find('Smax').text = str(SS_max)
                    colour.find('Vmax').text = str(SV_max)
                    colour.find('Hmin').text = str(SH_min)
                    colour.find('Smin').text = str(SS_min)
                    colour.find('Vmin').text = str(SV_min)
                    colour.find('Erosion').text = str(SErode)
                    colour.find('Dilation').text = str(SDilate)
            tree.write(config)
            cv2.destroyAllWindows()
            break
        
        
    #End Calibration
    cv2.destroyAllWindows()




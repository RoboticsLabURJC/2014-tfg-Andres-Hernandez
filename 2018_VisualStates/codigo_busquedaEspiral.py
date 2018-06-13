numVuelta=numVuelta+1
if(numVuelta>100 and numVuelta < 120):
    timerW=timerW+(timerW/8)
    if(numVuelta==119):
        numVuelta=0
    if(wSearch<1 and numVuelta==101):
       wSearch=wSearch+0.2
       numIteracionesGreen=0
       numIteracionesOrange=0
else:
    
self.interfaces.myCmdvel.sendCMDVel(1.8+wSearch,0,0,0,0,1.5 - wSearch)

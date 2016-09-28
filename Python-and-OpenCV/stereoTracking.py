# -*- coding: utf-8 -*-
"""
Created on Thu Sep 22 11:25:41 2016

@author: Dennis
"""

import numpy as np
import cv2
import json
import glob
import os

class stereoTracking():
    '''
    To Do:
    Funktioner att implementera:
    - def saveMatchImg: Spara en bild av de slutgiltiga matchningarna för varje tidpunkt.'''
    
    def __init__(self):
        self.u = 3.75e-06 #Pixel size. From documentation.
        self.baseline = 0.24 #In m, baseline between two cameras. From documentation.
        self.f = 3.8e-03 #In m, focal length. From Documentation.
        self.scaling = 1 #Scaling factor. Picked from thin air.
        with open("data.json", "r") as f:
            data = json.load(f)
        self.camera_matrix = np.asarray(data['camera_matrix'],dtype='float64')
        self.dist = np.asarray([data['dist_coeff']],dtype='float64')
        self.mtx = np.asarray(data['mtx'],dtype='float64')
        
    def rbgSplitter(self):
        '''Delar upp RBG bilder till tre enskilda bilder (left,mid,right).
        Input: Ingen. Läser in alla bilder i wdir.
        Output: Skapar tre bilder i wdir och tar bort ursprungsbilden.
        Anmärkningar:'''
        images = glob.glob('*.png')
        for iNames in images:
            img = cv2.imread(iNames,1)
            l,m,r = cv2.split(img)
            ln = iNames[:31] + str('-left.png')
            mn = iNames[:31] + str('-mid.png')
            rn = iNames[:31] + str('-right.png')
            cv2.imwrite(ln,l)
            cv2.imwrite(mn,m)
            cv2.imwrite(rn,r)
            os.remove(iNames)
    
    def undistort(self):  
        '''Tar bort distortions från alla bilder i wdir.
        Input: Ingen. Tar parametrar från __init__.
        Output: Undistorted images.
        Anmärkningar: Kamera parametrana kan vara felaktiga.
        To Do:
        - Ta reda på om kamera parametrarna är korrekta.'''
        images = glob.glob('*.png')
        for iName in images:
            img = cv2.imread(iName)
            h, w = img.shape[:2]
            newcameramtx, roi=cv2.getOptimalNewCameraMatrix(self.mtx,self.dist,(w,h),1,(w,h)) # undistort
            mapx,mapy = cv2.initUndistortRectifyMap(self.mtx,self.dist,None,newcameramtx,(w,h),5)
            dst = cv2.remap(img,mapx,mapy,cv2.INTER_LINEAR) # crop the image
            x,y,w,h = roi
            dst = dst[y:y+h, x:x+w]
            os.remove(iName) #save image and remove old
            cv2.imwrite(iName,dst)
        
    def featureDetection(self, timepoint):
        '''Använder ORB för tillfället, verkar vara ett bra alternativ.
        Input: En timepoint. Siffrorna efter tidpunkt i nuvarande bildnamngivning.
        Output: Descriptors och keypoints för vänster och höger kamera vid
        tidpunkt t0 (timepoint) och t1 (timepoint+1)
        Anmärkningar: Tror inte denna behöver ändras.'''
        t0 = str(timepoint)
        t1 = str(timepoint+1)
        while len(t0) < 4:
            t0 = '0'+t0
        while len(t1) < 4:
            t1 = '0'+t1
        lIP = cv2.imread(glob.glob('*'+t0+'-left.png')[0],0)
        rIP = cv2.imread(glob.glob('*'+t0+'-right.png')[0],0)
        lIC = cv2.imread(glob.glob('*'+t1+'-left.png')[0],0)
        rIC = cv2.imread(glob.glob('*'+t1+'-right.png')[0],0)
        orb = cv2.ORB_create()
        kp1, des1 = orb.detectAndCompute(lIP,None)
        kp2, des2 = orb.detectAndCompute(rIP,None)
        kp3, des3 = orb.detectAndCompute(lIC,None)
        kp4, des4 = orb.detectAndCompute(rIC,None)
        return des1,des2,des3,des4,kp1,kp2,kp3,kp4
        
    def coordinatesCalc(self,ptsl,ptsr):
        '''Beräknar X, Y och Z koordinater för alla features.
        Input: En lista med x, y koordinater för alla features matchade
        i höger och vänster bild.
        Output: En lista med X, Y och Z koordinater för alla features.
        Anmärkningar: Kallas två gånger, en gång för t0 och en gång för t1.
        To do:
        - Den borde kolla storleken på bilderna och inte använda
        fasta värden.
        - Använda flera kamera par för att beräkna djup och felmarginal.'''
        coordinates = []
        c = 0
        length = len(ptsl)
        while c < length:
            s = ptsl[c][0] - ptsr[c][0]
            xl = (ptsl[c][0]-526.5)*self.u
            yl = -(ptsl[c][1]-353)*self.u
            Z = (self.baseline*self.f)/(s*self.u*self.scaling)
            v1x = np.arctan(self.f/xl)
            X = np.tan(np.pi/2-v1x)*Z
            v1y = np.arctan(self.f/yl)
            Y = np.tan(np.pi/2-v1y)*Z
            temp = []
            temp.append(X)
            temp.append(Y)
            temp.append(Z)
            coordinates.append(temp)
            np.shape(coordinates)
            c += 1
        return coordinates

    def updatePosition(self,oldPosition, R, t):
        '''Uppdaterar positionen för nuvarande tidpunkt.
        Input: Sist kända position (vektor), rotationsmatrisen och translations vektorn.
        Output: Nuvarande position.
        Anmärkningar: Finns flera sätt att beräkna detta. Nuvarande implementation
        är troligen en av de sämsta.
        To do: 
        - Implementera en mycket mer korrekt beräkningsmetod.
        - Kan man få en felmarginal?'''
        newPosition = oldPosition+t
        return newPosition
    
    #Methods
    def multiMatch(self,des1,des2,des3,des4,kp1,kp2,kp3,kp4,tp):
        '''Tar bort de features som inte är med på alla bilder (t0 och t1).
        Input: Descriptors och keypoints för alla bilder.
        Output: Numpy arrays med x, y koordinater för de features som är med
        i alla bilder.
        Anmärkningar: Matchar först L med R för t0 och t1 och använder sedan
        kvarvarande descriptors mellan Lt0 och Lt1. (Vänstra kameran ses som
        center för världen.)
        To do:
        - Snyggare kod...
        - Implementera version för alla tre kameror.
        - Borde kalla på en funktion som gör bilder av den slutgiltiga matchningen.'''
        bf  = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matchest0 = bf.match(des1,des2)
        matchest1 = bf.match(des3,des4)
        tmpt0, tmpt1, ttk1, ttk2, ttk3, ttk4, tk1, tk2, tk3, tk4, mD0, mD1 = [],[],[],[],[],[],[],[],[],[],[],[]
        matchest0 = sorted(matchest0, key = lambda x:x.distance)
        matchest1 = sorted(matchest1, key = lambda x:x.distance)
        x,y,c = 0,0,0
        while x < 20:
            tmpt0.append(matchest0[c])
            x = matchest0[c].distance
            c += 1
        c = 0
        while y < 20:
            tmpt1.append(matchest1[c])
            y = matchest1[c].distance
            c += 1
        for m in tmpt0:
            mD0.append(des1[m.queryIdx])
            ttk1.append(kp1[m.queryIdx])
            ttk2.append(kp2[m.trainIdx])
        for n in tmpt1:
            mD1.append(des3[n.queryIdx])
            ttk3.append(kp3[n.queryIdx])
            ttk4.append(kp4[n.trainIdx])
        mD0 = np.uint8(mD0)
        mD1 = np.uint8(mD1)
        mMatch = bf.match(mD0,mD1)
        for m in mMatch:
            tk1.append(ttk1[m.queryIdx].pt)
            tk2.append(ttk2[m.queryIdx].pt)
            tk3.append(ttk3[m.trainIdx].pt)
            tk4.append(ttk4[m.trainIdx].pt)
        tk1,tk2,tk3,tk4 = np.float32(tk1),np.float32(tk2),np.float32(tk3),np.float32(tk4)
#        self.whatever(ttk1,ttk3,tk1,tk3,mD0,mD1,tp)
        return tk1,tk2,tk3,tk4
    
    def whatever(self,kp1,kp3,pts1,pts3,md1,md3,tp):
        F, mask = cv2.findFundamentalMat(pts1,pts3,cv2.RANSAC)
        c = 0
        fkp1,fkp3,fmd1,fmd3 = [],[],[],[]
        for m in mask:
            if m == 1:
                fkp1.append(kp1[c])
                fkp3.append(kp3[c])
                fmd1.append(md1[c])
                fmd3.append(md3[c])
        bf  = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        fmatches = bf.match(md1,md3)
        print len(fmatches)
        print len(fkp1),len(fkp3),len(fmd1),len(fmd3)
        ci = cv2.imread(glob.glob('*'+str(tp)+'-left.png')[0],0)
        ni = cv2.imread(glob.glob('*'+str(tp+1)+'-left.png')[0],0)
        matchedImage = cv2.drawMatches(ci,fkp1,ni,fkp3,fmatches,None,flags=2)
        cv2.imwrite('finalMatches-'+str(tp)+'.png',matchedImage)
        
    def selectInliers(self,pts1,pts2,pts3,pts4):
        '''Använder RANSAC för att ta bort outliers.
        Input: Numpy arrays med x, y koordinater för alla matchade features.
        Output: Numpy arrays med bara inlier koordinater.
        Anmärkning: Onödig? Verkar aldrig sortera bort nått?
        To Do:
        - Kontrollera om den behövs.
        - Görs detta verkligen på x, y koordinaterna? Kanske görs bättre på XYZ?
        - Snygga till koden.'''
        F1, mask1 = cv2.findFundamentalMat(pts1,pts2,cv2.RANSAC)
        F2, mask2 = cv2.findFundamentalMat(pts3,pts4,cv2.RANSAC)
        c = 0
        l = len(pts1)
        t1,t2,t3,t4 = [],[],[],[]
        while c < l:
            tmp1,tmp2,tmp3,tmp4 = [],[],[],[]
            if mask1[c][0] == 1 and mask2[c][0] == 1:
                for m in pts1[c]:
                    tmp1.append(m)
                for m in pts2[c]:
                    tmp2.append(m)
                for m in pts3[c]:
                    tmp3.append(m)
                for m in pts4[c]:
                    tmp4.append(m)
                t1.append(tmp1)
                t2.append(tmp2)
                t3.append(tmp3)
                t4.append(tmp4)
            c += 1
        t1 = np.float32(t1)
        t2 = np.float32(t2)
        t3 = np.float32(t3)
        t4 = np.float32(t4)
        return t1,t2,t3,t4

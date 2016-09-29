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
    '''To Do:
    Funktioner att implementera:
    - def saveMatchImg: Spara en bild av de slutgiltiga matchningarna för varje tidpunkt.
    - def plotTrajectory: Plotta trajectory, gärna med bilder av matchningarna på sidan.'''
    
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
        self.beeCameraK = [[0.751683, 0, 0.499852], [0, 1.00224, 0.490084], [0, 0, 1]]
        
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
        t2 = str(timepoint+2)
        t3 = str(timepoint+3)
        while len(t0) < 4:
            t0 = '0'+t0
        while len(t1) < 4:
            t1 = '0'+t1
        while len(t2) < 4:
            t2 = '0'+t2
        while len(t3) < 4:
            t3 = '0'+t3
        lIP = cv2.imread(glob.glob('*'+t0+'-left.png')[0],0)
        rIP = cv2.imread(glob.glob('*'+t1+'-left.png')[0],0)
        lIC = cv2.imread(glob.glob('*'+t2+'-left.png')[0],0)
        rIC = cv2.imread(glob.glob('*'+t3+'-left.png')[0],0)
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
            Y = np.tan(np.pi/2-v1y)*Z #Alternativt: Y = (yl*Z)/self.f
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

    def multiMatchFundamental(self,des1,des2,des3,des4,kp1,kp2,kp3,kp4,tp):
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
        tp1,tp2,tp3,tp4 = [],[],[],[]
        firstKpMatches1,firstKpMatches2,firstKpMatches3,firstKpMatches4 = [],[],[],[]
        firstDesMatches1,firstDesMatches2,firstDesMatches3,firstDesMatches4 = [],[],[],[]
        for m in matchest0:
            firstKpMatches1.append(kp1[m.queryIdx])
            firstDesMatches1.append(des1[m.queryIdx])
            tp1.append(kp1[m.queryIdx].pt)
            firstKpMatches2.append(kp2[m.trainIdx])
            firstDesMatches2.append(des2[m.trainIdx])
            tp2.append(kp2[m.trainIdx].pt)
        for n in matchest1:
            firstKpMatches3.append(kp3[n.queryIdx])
            firstDesMatches3.append(des3[n.queryIdx])
            tp3.append(kp3[n.queryIdx].pt)
            firstKpMatches4.append(kp4[n.trainIdx])
            firstDesMatches4.append(des4[n.trainIdx])
            tp4.append(kp4[n.trainIdx].pt)
        F1, mask1 = cv2.findFundamentalMat(np.float32(tp1),np.float32(tp2),cv2.RANSAC,2,0.999)
        F2, mask2 = cv2.findFundamentalMat(np.float32(tp3),np.float32(tp4),cv2.RANSAC,2,0.999)
        maskedKp1,maskedKp2,maskedKp3,maskedKp4,maskedDes1,maskedDes2,maskedDes3,maskedDes4 = [],[],[],[],[],[],[],[]
        c = 0
        for m in mask1:
            if m == 1:
                maskedKp1.append(firstKpMatches1[c])
                maskedDes1.append(firstDesMatches1[c])
                maskedKp2.append(firstKpMatches2[c])
                maskedDes2.append(firstDesMatches2[c])
            c += 1
        c = 0
        for n in mask2:
            if n == 1:
                maskedKp3.append(firstKpMatches3[c])
                maskedDes3.append(firstDesMatches3[c])
                maskedKp4.append(firstKpMatches4[c])
                maskedDes4.append(firstDesMatches4[c])
            c += 1
        mMatches = bf.match(np.uint8(maskedDes1),np.uint8(maskedDes3))
        spt1,spt2,spt3,spt4, secondKpMatches1,secondKpMatches3,secondDesMatches1,secondDesMatches3 = [],[],[],[],[],[],[],[]
        for m in mMatches:
            spt1.append(maskedKp1[m.queryIdx].pt)
            spt2.append(maskedKp2[m.queryIdx].pt)
            secondKpMatches1.append(maskedKp1[m.queryIdx])
            secondDesMatches1.append(maskedDes1[m.queryIdx])
            spt3.append(maskedKp3[m.trainIdx].pt)
            spt4.append(maskedKp4[m.trainIdx].pt)
            secondKpMatches3.append(maskedKp3[m.trainIdx])
            secondDesMatches3.append(maskedDes3[m.trainIdx])
        F3, mask3 = cv2.findFundamentalMat(np.float32(spt1),np.float32(spt3),cv2.RANSAC,2,0.999)
        fpts1, fpts2, fpts3, fpts4, finalKp1,finalKp3,finalDes1,finalDes3 = [],[],[],[],[],[],[],[]
        c = 0
        for m in mask3:
            if m == 1:
                finalKp1.append(secondKpMatches1[c])
                finalDes1.append(secondDesMatches1[c])
                fpts1.append(spt1[c])
                fpts2.append(spt2[c])
                finalKp3.append(secondKpMatches3[c])
                finalDes3.append(secondDesMatches3[c])
                fpts3.append(spt3[c])
                fpts4.append(spt4[c])
            c += 1
        finalDes1, finalDes3 = np.uint8(finalDes1), np.uint8(finalDes3)
        imgMatch = bf.match(finalDes1,finalDes3)
        ci = cv2.imread(glob.glob('*'+str(tp)+'-left.png')[0],0)
        ni = cv2.imread(glob.glob('*'+str(tp+2)+'-left.png')[0],0)
        matchedImage = cv2.drawMatches(ci,finalKp1,ni,finalKp3,imgMatch,None,flags=2)
        cv2.imwrite('finalMatches-'+str(tp)+'.png',matchedImage)
        return np.float32(fpts1),np.float32(fpts2),np.float32(fpts3),np.float32(fpts4)
    
    def multiMatchHomography(self,des1,des2,des3,des4,kp1,kp2,kp3,kp4,tp):
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
        tp1,tp2,tp3,tp4 = [],[],[],[]
        firstKpMatches1,firstKpMatches2,firstKpMatches3,firstKpMatches4 = [],[],[],[]
        firstDesMatches1,firstDesMatches2,firstDesMatches3,firstDesMatches4 = [],[],[],[]
        for m in matchest0:
            firstKpMatches1.append(kp1[m.queryIdx])
            firstDesMatches1.append(des1[m.queryIdx])
            tp1.append(kp1[m.queryIdx].pt)
            firstKpMatches2.append(kp2[m.trainIdx])
            firstDesMatches2.append(des2[m.trainIdx])
            tp2.append(kp2[m.trainIdx].pt)
        for n in matchest1:
            firstKpMatches3.append(kp3[n.queryIdx])
            firstDesMatches3.append(des3[n.queryIdx])
            tp3.append(kp3[n.queryIdx].pt)
            firstKpMatches4.append(kp4[n.trainIdx])
            firstDesMatches4.append(des4[n.trainIdx])
            tp4.append(kp4[n.trainIdx].pt)
        M1, mask1 = cv2.findHomography(np.float32(tp1), np.float32(tp2), cv2.RANSAC,3.0)
        M2, mask2 = cv2.findHomography(np.float32(tp3), np.float32(tp4), cv2.RANSAC,3.0)
        maskedKp1,maskedKp2,maskedKp3,maskedKp4,maskedDes1,maskedDes2,maskedDes3,maskedDes4 = [],[],[],[],[],[],[],[]
        c = 0
        for m in mask1:
            if m == 1:
                maskedKp1.append(firstKpMatches1[c])
                maskedDes1.append(firstDesMatches1[c])
                maskedKp2.append(firstKpMatches2[c])
                maskedDes2.append(firstDesMatches2[c])
            c += 1
        c = 0
        for n in mask2:
            if n == 1:
                maskedKp3.append(firstKpMatches3[c])
                maskedDes3.append(firstDesMatches3[c])
                maskedKp4.append(firstKpMatches4[c])
                maskedDes4.append(firstDesMatches4[c])
            c += 1
        mMatches = bf.match(np.uint8(maskedDes1),np.uint8(maskedDes3))
        spt1,spt2,spt3,spt4, secondKpMatches1,secondKpMatches3,secondDesMatches1,secondDesMatches3 = [],[],[],[],[],[],[],[]
        for m in mMatches:
            spt1.append(maskedKp1[m.queryIdx].pt)
            spt2.append(maskedKp2[m.queryIdx].pt)
            secondKpMatches1.append(maskedKp1[m.queryIdx])
            secondDesMatches1.append(maskedDes1[m.queryIdx])
            spt3.append(maskedKp3[m.trainIdx].pt)
            spt4.append(maskedKp4[m.trainIdx].pt)
            secondKpMatches3.append(maskedKp3[m.trainIdx])
            secondDesMatches3.append(maskedDes3[m.trainIdx])
        M3, mask3 = cv2.findHomography(np.float32(spt1), np.float32(spt3), cv2.RANSAC,3.0)
        fpts1, fpts2, fpts3, fpts4, finalKp1,finalKp3,finalDes1,finalDes3 = [],[],[],[],[],[],[],[]
        c = 0
        for m in mask3:
            if m == 1:
                finalKp1.append(secondKpMatches1[c])
                finalDes1.append(secondDesMatches1[c])
                fpts1.append(spt1[c])
                fpts2.append(spt2[c])
                finalKp3.append(secondKpMatches3[c])
                finalDes3.append(secondDesMatches3[c])
                fpts3.append(spt3[c])
                fpts4.append(spt4[c])
            c += 1
        finalDes1, finalDes3 = np.uint8(finalDes1), np.uint8(finalDes3)
        imgMatch = bf.match(finalDes1,finalDes3)
        ci = cv2.imread(glob.glob('*'+str(tp)+'-left.png')[0],0)
        ni = cv2.imread(glob.glob('*'+str(tp+2)+'-left.png')[0],0)
        matchedImage = cv2.drawMatches(ci,finalKp1,ni,finalKp3,imgMatch,None,flags=2)
        cv2.imwrite('finalMatchesHomography-'+str(tp)+'.png',matchedImage)
        return np.float32(fpts1),np.float32(fpts2),np.float32(fpts3),np.float32(fpts4)

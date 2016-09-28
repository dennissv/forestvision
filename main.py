# -*- coding: utf-8 -*-
"""
Created on Wed Sep 28 13:39:20 2016

@author: Dennis
"""

#Imports
import numpy as np
import stereoTracking
import cv2
st = stereoTracking.stereoTracking()

#Variable declerations
t0 = 0 #Start tid
t1 = 13 #Slut tid
timePoint = t0
position = np.float32([0,0,0])
trajectory = []

#Image manipulation
#st.rbgSplitter()
#st.undistort()

#Main loop
while timePoint < t1:
    tP = []
    des1,des2,des3,des4,kp1,kp2,kp3,kp4 = st.featureDetection(timePoint)
    pts1,pts2,pts3,pts4 = st.multiMatch(des1,des2,des3,des4,kp1,kp2,kp3,kp4,timePoint)
    pts1,pts2,pts3,pts4 = st.selectInliers(pts1,pts2,pts3,pts4)
    coordinates1 = np.float32(st.coordinatesCalc(pts1,pts2))
    coordinates2 = np.float32(st.coordinatesCalc(pts3,pts4))
    a,b,c = cv2.estimateAffine3D(coordinates1,coordinates2,confidence=0.99)
    R = b[:,[0,1,2]]
    t = b[:,3]
    position += t
    tP.append(position[0])
    tP.append(position[1])
    tP.append(position[2])
    trajectory.append(tP)
    timePoint += 1
print position
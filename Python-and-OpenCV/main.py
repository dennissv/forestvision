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
t1 = 1 #Slut tid
beeCameraK = [[0.751683, 0, 0.499852], [0, 1.00224, 0.490084], [0, 0, 1]]
#K = np.float32([[724.37, 0, 705.72],[0,708.99,450.38],[0,0,1]])
#K.shape = (3,3)
timePoint = t0
position = np.float32([1,1,1])
position.shape = (3,1)
trajectory = []

#Image manipulation
#st.rbgSplitter()
#st.undistort()

#Main loop
while timePoint < t1:
    des1,des2,des3,des4,kp1,kp2,kp3,kp4 = st.featureDetection(timePoint)
    pts1,pts2,pts3,pts4 = st.multiMatchFundamental(des1,des2,des3,des4,kp1,kp2,kp3,kp4,timePoint)
    coordinates1, coordinates2 = st.coordinatesCalc(pts1,pts2), st.coordinatesCalc(pts3,pts4)
    coordinates1, coordinates2 = st.imuSupport(coordinates1, coordinates2)
    a,b,c = cv2.estimateAffine3D(coordinates1,coordinates2,confidence=0.999)
    R = b[:,[0,1,2]]
    t = b[:,3]
    t.shape = (3,1)
#    position = np.dot(beeCameraK,np.dot(R,position))+t
    position = st.updatePosition(position, R, t)
    trajectory.append([position[0], position[1], position[2]])
    timePoint += 1
print position

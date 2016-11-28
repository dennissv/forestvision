# -*- coding: utf-8 -*-
"""
Created on Wed Nov 23 15:34:25 2016

@author: Dennis
"""

import numpy as np
import pylab as pl
import transformations
from geopy.distance import vincenty

def readINS(filename):
    results = []
    f = open(filename, 'r')
    c = 0
    for line in f:
        if c < 31:
            c += 1
        else:
            current = []
            for i in line.split(' '):
                if not i == '':
                    try:
                        current.append(float(i.strip('\n')))
                    except:
                        None
            if len(current)>0:
                results.append(current)
    f.close()    
    return np.float64(results)
    
def readCamera(filename):
    results = []
    f = open(filename, 'r')
    first = True
    for line in f:
        if not first:
            s = map(float, line.split('\t'))
            current = []
            for i in s[3:10]:
                current.append(i)
            current.append(s[12])
            current.append(s[13])
            current.append(s[14])
            results.append(current)
        first = False
    f.close()    
    return np.float64(results)
    
def usefulINS(data):
    '''
    Time (s) | North (m) | East | Altitude | Roll (radians) | Pitch | Yaw
    '''
    results,raw = [],[]
    n0 = data[:,1][0]
    e0 = data[:,0][0]
    for i in data:
        raw.append([i[4]%86400, i[1], i[0], i[3], i[10], \
                        i[11], i[12]])
        results.append([i[4]%86400, i[1]-n0, i[0]-e0, i[3], i[10]*(np.pi/180), \
                        i[11]*(np.pi/180), i[12]*(np.pi/180)])
    return np.float64(results),np.float64(raw)
    
def usefulINSNewFormat(data):
    '''
    Time (s) | North (m) | East | Altitude | Roll (radians) | Pitch | Yaw
    '''
    results,raw = [],[]
    n0 = data[:,10][0]
    e0 = data[:,9][0]
    for i in data:
        raw.append([i[18]%86400, i[10], i[9], i[1], i[0], \
                        i[2], i[4], i[13],])
        results.append([i[18]%86400, i[10]-n0, i[9]-e0, i[11], i[0]*(np.pi/180), \
                        i[2]*(np.pi/180), i[4]*(np.pi/180),i[13], i[1], i[3], i[5]])
    return np.float64(results),np.float64(raw)

def fixCameraTime(data):
    for i in range(len(data[:,0])):
        data[:,0][i] += 17
    return data
    
def setStartAltitudeCamera(cam,ins):
    for i in range(len(cam[:,0])):
        cam[:,3][i] += ins[:,3][i]
    return cam
    
def syncINStoCamera(cam,ins):
    newINS = []
    for i in cam[:,0]:
        values = [999,998]
        c = 0
        while values[1] < values[0]:
            values[0] = values[1]
            values[1] = abs(i-ins[:,0][c])
            c += 1
        newINS.append(ins[c-2])
        try:
            ins = ins[c-2:]
        except:
            None
    return np.float64(newINS)

def rotateVector(dvec, eA):
    dvec = np.float64(dvec)
    r = transformations.euler_matrix(eA[0]+np.pi,0,0,axes='sxzy')
    rot = np.vstack((np.vstack((r[0][:3],r[1][:3])),r[2][:3]))
    dvec = np.dot(dvec,rot)
    r = transformations.euler_matrix(0,eA[1],0,axes='sxzy')
    rot = np.vstack((np.vstack((r[0][:3],r[1][:3])),r[2][:3]))
    dvec = np.dot(dvec,rot)
    r = transformations.euler_matrix(0,0,eA[2]-np.pi/2,axes='sxzy')
    rot = np.vstack((np.vstack((r[0][:3],r[1][:3])),r[2][:3]))
    return np.dot(dvec, rot)
    
def rotateCamera(cam, ins):
    walk = []
    result = []
    for i in range(len(cam[:,0])):
        vec = np.float64([cam[:,7][i],cam[:,8][i],cam[:,9][i]])
        eulerAngles = [ins[:,4][i],-0.45-ins[:,5][i],ins[:,6][i]]
        walk.append(rotateVector(vec,eulerAngles))
    for i in range(len(walk)):
        if i == 0:
            result.append([cam[:,0][i],walk[i][0],walk[i][1]+cam[:,3][0],walk[i][2],cam[:,4][i], \
                          cam[:,5][i], cam[:,6][i]])
        else:
            result.append([cam[:,0][i],result[i-1][1]+walk[i][0],result[i-1][2]+walk[i][1]+0.0135, \
                          result[i-1][3]+walk[i][2],cam[:,4][i],cam[:,5][i],cam[:,6][i]])
    return np.float64(result)
    
def distance(data):
    dist = 0
    for i in range(len(data)-1):
        dist += np.sqrt((data[:,1][i+1]-data[:,1][i])**2 + (data[:,2][i+1]-data[:,2][i])**2 \
                        + (data[:,3][i+1]-data[:,3][i])**2)
    return dist
    
def convertCameratoUTM(cam, rins):
    conv = []
    for i in range(len(cam[:,0])):
        conv.append([cam[:,0][i],cam[:,1][i]+rins[:,1][0],cam[:,2][i],\
                     cam[:,3][i]+rins[:,2][0],cam[:,4][i],cam[:,5][i],\
                     cam[:,6][i]])
    return np.float64(conv)
    
def stackAndWrite(cam, ins, filename):
    #Time, Cam xyz, INS xyz, INS angles
    combined = []
    for i in range(len(ins[:,0])):
        combined.append([cam[:,0][i],cam[:,1][i],cam[:,2][i],cam[:,3][i],ins[:,1][i],\
                        ins[:,2][i],ins[:,3][i],ins[:,4][i],ins[:,5][i],ins[:,6][i]])
    np.savetxt(filename,np.float64(combined),delimiter=',')

def finalExport(cam, ins, filename):
    cv = [[0,0,0]]
    dv = [[0,0,0]]
    final = []
    for i in range(1,len(cam[:,0])):
        dt = cam[:,0][i]-cam[:,0][i-1]
        cv.append([(cam[:,1][i]-cam[:,1][i-1])/dt,(cam[:,3][i]-cam[:,3][i-1])/dt,\
                   (cam[:,2][i]-cam[:,2][i-1])/dt])
        dv.append([(cam[:,4][i]-cam[:,4][i-1])/dt,(cam[:,5][i]-cam[:,5][i-1])/dt,\
                   (cam[:,6][i]-cam[:,6][i-1])/dt])
    for i in range(len(cv)):
        final.append([cam[:,0][i],ins[:,1][i],ins[:,2][i],ins[:,3][i],ins[:,7][i],\
                      cv[i][0],cv[i][1],cv[i][2],ins[:,4][i],ins[:,5][i],ins[:,6][i],\
                      ins[:,8][i],ins[:,9][i],ins[:,10][i],dv[i][0],dv[i][1],dv[i][2]])
    np.savetxt(filename,np.float64(final),delimiter=',')

#wat = readINS('long1125verbose.txt')
ins, raw = usefulINSNewFormat(readINS('long1125verbose.txt')) #'sluINS1123.txt'
camera = fixCameraTime(readCamera('long1125.txt')) #'slu1123(3).txt'
camera = camera[:10525]
nins = syncINStoCamera(camera, ins)
nraw = syncINStoCamera(camera, raw)
camera = rotateCamera(setStartAltitudeCamera(camera,nins),nins)
insdist = distance(ins)
ncdist = distance(camera)
ccam = convertCameratoUTM(camera,nraw)
stackAndWrite(ccam,nraw,'slu1125_out_kalman.csv')
finalExport(camera,nins,'long1125_kalman.csv')

start = (63.821214,20.314274)
port = (63.820764, 20.312653)
slut = (63.821345,20.311249)
wy = vincenty(start,(slut[0],start[1])).m
wx = -vincenty(start,(start[0],slut[1])).m
xy = -vincenty(start,(port[0],start[1])).m
xx = -vincenty(start,(start[0],port[1])).m

fig1 = pl.figure(figsize=(10,10))
ins_plot = pl.plot(nins[:,2],nins[:,1],'go',label='INS solution')
newc_plot = pl.plot(camera[:,3],camera[:,1],'bo',label='Camera solution')
pl.xlabel('West - East', fontsize=18)
pl.ylabel('South - North', fontsize=18)
pl.legend(loc='upper right')
pl.show()

fig2 = pl.figure(figsize=(10,10))
altitudei = pl.plot(np.linspace(0,len(nins[:,0]),len(nins[:,0])),nins[:,3],'go',label='INS solution')
altitudec = pl.plot(np.linspace(0,len(camera[:,0]),len(camera[:,0])),camera[:,2],'bo',label='Camera solution')
pl.xlabel('Frame', fontsize=18)
pl.ylabel('Altitude', fontsize=18)
pl.legend(loc='upper right')
pl.show()
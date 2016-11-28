# -*- coding: utf-8 -*-
"""
Created on Thu Nov 24 16:20:32 2016

@author: Dennis
"""

import numpy as np
import pylab as pl
import transformations

def readCamera(filename):
    results = []
    f = open(filename, 'r')
    first = True
    for line in f:
        if not first:
            split1 = map(float, line.split('\t'))
            results.append(split1[3:10])
        first = False
    f.close()    
    return np.float64(results)
    
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
    
def usefulINS(data):
    '''
    Time (s) | North (m) | East | Altitude | Roll (degrees) | Pitch | Yaw
    '''
    results = []
    n0 = data[:,1][0]
    e0 = data[:,0][0]
    for i in data:
        results.append([i[4]%86400, i[1]-n0, i[0]-e0, i[3], i[10], i[11], i[12]])
    return np.float64(results)
    
def fixCameraTime(data):
    for i in range(len(data[:,0])):
        data[:,0][i] += 17
    return data
    
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
    
def xzCamera(cam, ins):
    newcam = []
    x,y,z = [],[],[]
    for i in range(len(cam[:,0])-1):
        v = np.sqrt((cam[:,1][i+1]-cam[:,1][i])**2+(cam[:,2][i+1]-cam[:,2][i])**2+\
                    (cam[:,3][i+1]-cam[:,3][i])**2)
        x.append(np.sin(ins[:,6][i]*(np.pi/180))*v)
        y.append(ins[:,3][i])
        z.append(np.cos(ins[:,6][i]*(np.pi/180))*v)
        if i == 1000:
            None
    for i in range(len(x)):
        if i == 0:
            newcam.append([cam[:,0][i],x[i],y[i],z[i],cam[:,[4]][i],cam[:,[5]][i],cam[:,[6]][i]])
        else:
            newcam.append([cam[:,0][i],newcam[i-1][1]+x[i],y[i],\
                           newcam[i-1][3]+z[i],cam[:,[4]][i],cam[:,[5]][i],cam[:,[6]][i]])
    newcam.append(newcam[len(newcam)-1])
    return np.float64(newcam)
    
def rotateVector(dvec, eA):
    dvec = np.float64(dvec)
    r = transformations.euler_matrix(eA[0],0,0,axes='sxzy')
    rot = np.vstack((np.vstack((r[0][:3],r[1][:3])),r[2][:3]))
    dvec = np.dot(dvec,rot)
    r = transformations.euler_matrix(0,-0.44-eA[1],0,axes='sxzy')
    rot = np.vstack((np.vstack((r[0][:3],r[1][:3])),r[2][:3]))
    dvec = np.dot(dvec,rot)
    r = transformations.euler_matrix(0,0,eA[2],axes='sxzy')
    rot = np.vstack((np.vstack((r[0][:3],r[1][:3])),r[2][:3]))
    return np.dot(dvec, rot)
    
def rotateCamera(cam, ins):
    walk = []
    result = []
    for i in range(1,len(cam[:,0])):
        vec = np.float64([(cam[:,1][i]-cam[:,1][i-1]),(cam[:,2][i]-cam[:,2][i-1]), \
                          (cam[:,3][i]-cam[:,3][i-1])])
        eulerAngles = [ins[:,4][i],ins[:,5][i]-0.4537,ins[:,6][i]]
        walk.append(rotateVector(vec,eulerAngles))
    for i in range(len(walk)):
        if i == 0:
            result.append(cam[:,0][i],walk[i][0],walk[i][1],walk[i][2],cam[:,4][i], \
                          cam[:,5][i], cam[:,6][i])
        else:
            result.append(cam[:,0][i],result[i-1][1]+walk[i][0],result[i-1][2]+walk[i][1], \
                          result[i-1][3]+walk[i][2],cam[:,4][i],cam[:,5][i],cam[:,6][i])
    return np.float64(result)

camera = fixCameraTime(readCamera('slu1123.txt'))
ins = syncINStoCamera(camera,usefulINS(readINS('sluINS1123.txt')))
yawCamera = xzCamera(camera,ins)

pl.figure(figsize=(10,10))
altitude = pl.plot(np.linspace(0,len(ins[:,0]),len(ins[:,0])),ins[:,3],'go')
pl.show()
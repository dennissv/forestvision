# -*- coding: utf-8 -*-
"""
Created on Wed Dec  7 00:16:31 2016

@author: Dennis
"""

import numpy as np
import transformations
import utm
import gmplot
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
            s = [float(i.strip('\n')) for i in line.split('\t')]
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
    n0 = data[:,10][0]
    e0 = data[:,9][0]
    for i in data:
        raw.append([i[18]%86400, i[10], i[9], i[1], i[0], \
                        i[2], i[4], i[13],])
        results.append([i[18]%86400, i[10]-n0, i[9]-e0, i[11], i[0]*(np.pi/180), \
                        i[2]*(np.pi/180), i[4]*(np.pi/180),i[13], i[1], i[3], i[5]])
    return np.float64(results),np.float64(raw)
    
def fixCameraTime(cam):
    '''Attempts to fix camera time'''
    for i in range(len(cam[:,0])):
        cam[:,0][i] += 19
    return cam
    
def setStartAltitudeCamera(cam,ins):
    for i in range(len(cam[:,0])):
        cam[:,3][i] += ins[:,3][i]
    return cam
    
def syncINStoCamera(cam,ins):
    '''Makes the camera and INS data the same length'''
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
    '''Rotates a camera vector according to world orientation'''
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
    '''Attempts to fix the camera path by rotating its vectors'''
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
            result.append([cam[:,0][i],result[i-1][1]+walk[i][0],result[i-1][2]+walk[i][1]+0.00535, \
                          result[i-1][3]+walk[i][2],cam[:,4][i],cam[:,5][i],cam[:,6][i]])
    return np.float64(result)

def convertCameratoUTM(cam, rins):
    '''Converts camera coordinates to UTM format'''
    conv = []
    for i in range(len(cam[:,0])):
        conv.append([cam[:,0][i],cam[:,1][i]+rins[:,1][0],cam[:,2][i],\
                     cam[:,3][i]+rins[:,2][0],cam[:,4][i],cam[:,5][i],\
                     cam[:,6][i]])
    return np.float64(conv)
    
def exportToKalman(cam, ins, filename):
    '''Writes camera and INS data to a .csv file to be read by the kalman filter'''
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
    final = np.array(final)
    np.savetxt(filename,final,fmt='%.15e',delimiter=',',\
               header='Time(s),INS_Northing(m),INS_Easting(m),INS_Altitude(m),INS_pos_Stdev (m),Cam_Vn(m/s),Cam_Ve(m/s),Cam_Va(m/s),INS_Roll (radians),INS_Pitch (radians),INS_Yaw (radians),INS_Roll_sd (radians),INS_Pitch_sd (radians),INS_Yaw_sd (radians),Cam_Vroll (radians),Cam_Vpitch (radians),Cam_Vyaw (radians)')

def convertToCoordinates(cam, ins):
    cins, ccam = [], []
    for i in range(len(cam[:,0])):
        cordins = utm.to_latlon(ins[:,2][i],ins[:,1][i],zone_number=34,\
                                         zone_letter='V')
        cordcam = utm.to_latlon(cam[:,3][i],cam[:,1][i],zone_number=34,\
                                         zone_letter='V')
        cins.append([cam[:,0][i],cordins[0],cordins[1]])
        ccam.append([cam[:,0][i],cordcam[0],cordcam[1]])
    return np.array(ccam),np.array(cins)
    
def readTimeStamps(filename):
    results = []
    f = open(filename, 'r')
    for line in f:
        s = [float(i.strip('\n')) for i in line.split('\t')]
        s = (s[3]-1)*60*60+s[4]*60+s[5]+17
        results.append(s)
    f.close()    
    return np.float64(results)
    
def evaluateWithTimestamps(cam, ins, timestamps, wps):
    camrmse, insrmse = 0, 0
    positions = []
    for c in range(len(timestamps)):
        dif = []
        for i in range(len(cam[:,0])):
            dif.append(abs(cam[:,0][i]-(timestamps[c]+4)))
        ind = dif.index(min(dif))
        camrmse += np.sqrt((vincenty((wps[c][0],wps[c][1]),(wps[c][0],cam[:,2][ind])).m)**2+\
                           (vincenty((wps[c][0],wps[c][1]),(cam[:,1][ind],wps[c][1])).m)**2)
        insrmse += np.sqrt((vincenty((wps[c][0],wps[c][1]),(wps[c][0],ins[:,2][ind])).m)**2+\
                           (vincenty((wps[c][0],wps[c][1]),(ins[:,1][ind],wps[c][1])).m)**2)
        positions.append(ins[ind])
    return camrmse, insrmse, np.float64(positions)

def makePlot(data1,data2,wps,tspos):
    gmap = gmplot.GoogleMapPlotter(63.821482, 20.311794, 16)
    gmap.plot(data1[:,1], data1[:,2], 'cornflowerblue', edge_width=3)
    gmap.plot(data2[:,1], data2[:,2], 'gold', edge_width=3)
    gmap.scatter(wps[:,0],wps[:,1], 'crimson', size=0.3, marker=False)
    gmap.scatter(tspos[:,1],tspos[:,2],'black', size=0.3, marker=False)
    gmap.plot
    gmap.draw("newtest1130.html")

def exportUnityPathINS(data, filename):
    f = open(filename,'w')
    for i in range(len(data[:,0])):
        f.write(str(data[:,2][i])+','+str(data[:,3][i])+','+str(data[:,1][i])+'\n')
    f.close()

def exportUnityPathCam(data, filename):
    f = open(filename,'w')
    for i in range(len(data[:,0])):
        f.write(str(data[:,3][i])+','+str(data[:,2][i])+','+str(data[:,1][i])+'\n')
    f.close()

#Variable declerations    
wps = np.array([[63.820783,20.312628],[63.821467,20.313170],[63.821569,20.313185],\
                [63.821678,20.313206],[63.821811,20.313260],[63.822064,20.313428],
                [63.822056,20.313716],[63.821714,20.314108],[63.821435,20.314212],
                [63.821075,20.314059],[63.820783,20.312628]])

#Main loop
ins, raw = usefulINS(readINS('ute1130ins.txt'))
camera = fixCameraTime(readCamera('ute1130cam.txt'))
camera = camera[:6700]
nins = syncINStoCamera(camera, ins)
nraw = syncINStoCamera(camera, raw)
camera = rotateCamera(setStartAltitudeCamera(camera,nins),nins)
ccam = convertCameratoUTM(camera,nraw)

ccam, cins = convertToCoordinates(ccam, nraw)
timestamps = readTimeStamps('1130timestamps.txt')[:11]
camrmse, insrmse, timespos = evaluateWithTimestamps(ccam, cins, timestamps, wps)
makePlot(ccam,cins,wps,timespos)
exportUnityPathINS(nins, 'ins.txt')
exportUnityPathCam(camera, 'camera.txt')
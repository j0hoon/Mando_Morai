#!/usr/bin/env python
#-*- coding:utf-8 -*-
import numpy as np
import math

parameters_GPS = {
    "X": 1.15, # meter
    "Y": 0,
    "Z": 0,
    "YAW": 0, #- 7.5*math.pi/180.0, # radian
    "PITCH": 0,
    "ROLL": 0
}

'''
cosR = math.cos(0)
cosP = math.cos(0)
cosY = math.cos(0)
sinR = math.sin(0)
sinP = math.sin(0)
sinY = math.sin(0)

rotRoll = np.array([1,0,0, 0,cosR,-sinR, 0,sinR,cosR]).reshape(3,3)
rotPitch = np.array([cosP,0,sinP, 0,1,0, -sinP,0,cosP]).reshape(3,3)
rotYaw = np.array([cosY,-sinY,0, sinY,cosY,0, 0,0,1]).reshape(3,3)
sensorRotationMat = rotYaw.dot(rotPitch.dot(rotRoll))
GPSPosition = np.array([parameters_GPS.get(i) for i in (["X","Y","Z"])])
sensorTranslationMat = np.array([GPSPosition])

print 'sensorRotationMat : ',sensorRotationMat
print 'sensorTranslationMat : ',sensorTranslationMat.T


Tr_sensor_to_vehicle = np.concatenate((sensorRotationMat,sensorTranslationMat.T),axis = 1)
Tr_sensor_to_vehicle = np.insert(Tr_sensor_to_vehicle, 3, values=[0,0,0,1],axis = 0)


print 'Tr : ',Tr_sensor_to_vehicle
print 'Tr shape : ',Tr_sensor_to_vehicle.shape
# 169.443771391 59.4660778968

mat = np.array([169.443771391, 59.4660778968,0,0]).reshape(4,1)
print 'mat shape : ',mat.shape


res = np.dot(Tr_sensor_to_vehicle,mat)
print 'res : ',res
'''

def translationMtx(x,y,z):
    M = np.array([[1,0,0,x],
                  [0,1,0,y],
                  [0,0,1,z],
                  [0,0,0,1],         
                          ])
    return M

def rotationMtx(yaw, pitch, roll): # no rotation

    R   = np.array([[1,0,0,0],
                    [0,1,0,0],
                    [0,0,1,0],
                    [0,0,0,1],
    ])
    return R

def transform_GPS2vehicle(params_GPS):
    GPS_pos = [params_GPS.get(i) for i in (["X","Y","Z"])]

    x_rel = -1*GPS_pos[0] 
    y_rel = -1*GPS_pos[1]
    z_rel = -1*GPS_pos[2]

    R_T = np.matmul(translationMtx(x_rel,y_rel,z_rel),rotationMtx(0.,0.,0.))
    R_T = np.matmul(R_T, rotationMtx(0.,0.,0.))

    R_T = np.linalg.inv(R_T)

    return R_T


mat = transform_GPS2vehicle(parameters_GPS)
print 'matrix : ',mat
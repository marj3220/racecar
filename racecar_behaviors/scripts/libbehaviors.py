#!/usr/bin/env python

import rospy
import cv2
import tf
import numpy as np
from tf.transformations import euler_from_quaternion

def quaternion_to_yaw(quat):
    # Uses TF transforms to convert a quaternion to a rotation angle around Z.
    # Usage with an Odometry message: 
    #   yaw = quaternion_to_yaw(msg.pose.pose.orientation)
    (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    return yaw
    
def multiply_transforms(trans1, rot1, trans2, rot2):
    trans1_mat = tf.transformations.translation_matrix(trans1)
    rot1_mat   = tf.transformations.quaternion_matrix(rot1)
    mat1 = np.dot(trans1_mat, rot1_mat)

    trans2_mat = tf.transformations.translation_matrix(trans2)
    rot2_mat    = tf.transformations.quaternion_matrix(rot2)
    mat2 = np.dot(trans2_mat, rot2_mat)

    mat3 = np.dot(mat1, mat2)
    trans3 = tf.transformations.translation_from_matrix(mat3)
    rot3 = tf.transformations.quaternion_from_matrix(mat3)
    
    return (trans3, rot3)

def brushfire(occupancyGrid):
    mapOfWorld = np.zeros(occupancyGrid.shape, dtype=int)
    
    mapOfWorld[occupancyGrid==100] = -1 # obstacles
    mapOfWorld[occupancyGrid==-1] = -5  # unknowns

    # do brushfire algorithm here

    for coord in zip(*np.where(mapOfWorld==-1)):
        neighbours = []
        neighbours.append([coord[0] - 1, coord[1]])
        neighbours.append([coord[0] + 1, coord[1]])
        neighbours.append([coord[0], coord[1] -1 ])
        neighbours.append([coord[0], coord[1] + 1])
        for neighbour in neighbours:
            if mapOfWorld[neighbour[0]][neighbour[1]] == 0:
                mapOfWorld[neighbour[0]][neighbour[1]] = 1

    DataToChange = True
    k = 1
    while DataToChange:
        DataToChange = False
        for coord in zip(*np.where(mapOfWorld==k)):
            DataToChange = True
            neighbours = []
            neighbours.append([coord[0] - 1, coord[1]])
            neighbours.append([coord[0] + 1, coord[1]])
            neighbours.append([coord[0], coord[1] -1 ])
            neighbours.append([coord[0], coord[1] + 1])
            for neighbour in neighbours:
                if mapOfWorld[neighbour[0]][neighbour[1]] == 0:
                    mapOfWorld[neighbour[0]][neighbour[1]] = k+1
        k=k+1

    for coord in zip(*np.where(mapOfWorld==-5)):
        mapOfWorld[coord[0]][coord[1]] = 0

    return mapOfWorld
    


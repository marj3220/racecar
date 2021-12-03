#!/usr/bin/env python

import cv2
import os
import numpy as np
import rospy

class PathFinder:
    def __init__(self, occupancyGrid) -> None:
        self.mapOfWorld = self.preface_map(occupancyGrid)

    def preface_map(self, occupancyGrid):
        map = np.zeros(occupancyGrid.shape, dtype=int)
        map[occupancyGrid==100] = -1 # obstacles
        map[occupancyGrid==-1] = -5  # unknowns
        return map
    
    def brushfire_one_pass(self):
        for coord in zip(*np.where(self.mapOfWorld==-1)):
                neighbours = []
                neighbours.append([coord[0] - 1, coord[1]])
                neighbours.append([coord[0] + 1, coord[1]])
                neighbours.append([coord[0], coord[1] -1 ])
                neighbours.append([coord[0], coord[1] + 1])
                for neighbour in neighbours:
                    if self.mapOfWorld[neighbour[0]][neighbour[1]] == 0:
                        self.mapOfWorld[neighbour[0]][neighbour[1]] = -1
    
    def brushfire(self, iterations):
        for _ in range(iterations):
            self.brushfire_one_pass()
        for coord in zip(*np.where(self.mapOfWorld==0)):
            self.mapOfWorld[coord[0]][coord[1]] = 10
        for coord in zip(*np.where(self.mapOfWorld==-5)):
            self.mapOfWorld[coord[0]][coord[1]] = 0
        
        return self.mapOfWorld
    
    def output_best_path(self):
        if not os.path.exists('output_directory'):
                os.makedirs('output_directory')
        result = cv2.imwrite(f'output_directory/trajectory.png', cv_image)
        if result:
            rospy.loginfo("Optimal trajectory has been calculated and saved!")
        else:
            rospy.logwarn('Optimal trajectory could not be saved!')

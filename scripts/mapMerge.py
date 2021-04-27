#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import sys
from matplotlib import pyplot as plt
import yaml

import os
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from numpy.lib.stride_tricks import as_strided

import roslaunch

class ROSMerge:
    def __init__(self):
        rospy.init_node('map_stitch',anonymous=False, log_level=rospy.DEBUG)
        self.map_pub = rospy.Publisher('map_merge/map', OccupancyGrid, queue_size=100)

    def main(self):
        self.m = MapStitching("maps/map_1.yaml", "maps/map_2.yaml")
        self.m.getInfo()

    def publishmsg(self):
        self.map_pub.publish(self.m.realogm)
        
        # m.outputfile()

class MapStitching:
    def __init__(self):
        self.outfile = str(sys.argv[3])
        self.grid_data = self.mapCallack(str(sys.argv[1]),str(sys.argv[2]))
        self.flag = False
        # self.mapCallack()

    def numpy_to_occupancy_grid(arr, info=None):
        if not len(arr.shape) == 2:
            raise TypeError('Array must be 2D')
        if not arr.dtype == np.int8:
            raise TypeError('Array must be of int8s')

        grid = OccupancyGrid()
        if isinstance(arr, np.ma.MaskedArray):
            # We assume that the masked value are already -1, for speed
            arr = arr.data
        grid.data = arr.ravel()
        grid.info = info or MapMetaData()
        grid.info.height = arr.shape[0]
        grid.info.width = arr.shape[1]

        return grid

    def mapCallack(self, file1, file2):
        if (not (os.path.exists(file1) and os.path.exists(file2))):
            self.flag = True
            return
        self.img1 = cv2.imread(file1, -1)
        self.img2 = cv2.imread(file2, -1)

        off1 = self.img1.shape[0] - 2048
        off2 = self.img1.shape[1] - 2048

        self.img1 = self.img1[off1:2048+off1, off2:2048+off2]
        print(self.img1.shape)
        print(self.img2.shape)
        
        self.filterRate = 0.2
        self.borderSize = 600
        self.free_thresh = 220
        self.occupied_thresh = 180

        bfMatch = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        orb = cv2.ORB_create()

        (self.kp1, des1) = orb.detectAndCompute(self.img1, None)
        (self.kp2, des2) = orb.detectAndCompute(self.img2, None)

        matches = bfMatch.match(des1, des2)
        matches = sorted(matches, key=lambda m: m.distance)

        keep = int(len(matches) * self.filterRate)
        self.matches = matches[:keep]

        pt1 = np.zeros((len(matches), 2))
        pt2 = np.zeros((len(matches), 2))

        for i, match in enumerate(matches):
            pt1[i] = self.kp1[match.queryIdx].pt
            pt2[i] = self.kp2[match.trainIdx].pt

        self.H, self.inlier = cv2.findHomography(pt2, pt1, method=cv2.RANSAC)

        self.rotation = 180.0 / np.pi * np.arctan2(self.H[0, 1], self.H[1, 1])
        self.transx = self.H[0, 2]
        self.transy = self.H[1, 2]
        self.scalex = np.sqrt(pow(self.H[0, 0], 2) + pow(self.H[0, 1], 2))
        self.scaley = np.sqrt(pow(self.H[1, 0], 2) + pow(self.H[1, 1], 2))

        self.outImg1 = self.getStitch()
        self.outImg1[self.outImg1 >= self.free_thresh] = 254
        self.outImg1[self.outImg1 <= self.occupied_thresh] = 0

        self.outImg1[:self.borderSize, :] = 205
        self.outImg1[self.outImg1.shape[0] - self.borderSize:, :] = 205
        self.outImg1[:, :self.borderSize] = 205
        self.outImg1[:, self.outImg1.shape[1] - self.borderSize:] = 205

        outputData = np.asarray(self.outImg1, dtype=np.int8) +128
        return outputData

    def getStitch(self):
        rows, cols = self.img2.shape[:2]
        imga = cv2.warpPerspective(self.img2, self.H, (rows, cols))
        image = cv2.addWeighted(imga, .5, self.img1, .5, 0.0, dtype=-1)
        return image

    def getDebug(self):
        outImg = None
        cv2.drawKeypoints(self.img1, self.kp1, self.img1, color=(255, 0, 0))
        cv2.drawKeypoints(self.img2, self.kp2, self.img2, color=(255, 0, 0))
        outImg = cv2.drawMatches(
            self.img1, self.kp1, self.img2, self.kp2, self.matches, None
        )
        plt.imshow(outImg, cmap='gray')
        plt.axis('off')
        plt.show()
        return outImg

    def getDump(self):
        plt.imshow(self.outImg1, cmap='gray')
        plt.show()

    def getInfo(self):
        print("Rotation: {}".format(self.rotation))
        print("Translation: (x, y): ({},{})".format(self.transx, self.transy))
        print("Matchers: {}".format(len(self.matches)))
        print("Inliers: {}".format(len(self.inlier)))
        print("Transform Matrix:\n{} \n".format(self.H))
    
    def outputfile(self):
        cv2.imwrite(self.outfile, self.outImg1)

if __name__ == "__main__":
    m = MapStitching()
    if (not m.flag):
        m.getInfo()
        m.outputfile()

    

   
    
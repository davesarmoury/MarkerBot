#!/usr/bin/env python3

import numpy as np
import cv2
import random

import sys
sys.path.insert(0, '../NCtoRobot')
from NCtoRobot.Kuka_SRC import Kuka_Src

w = 1920
h = 1080

cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, w)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, h)

def pix2Robot(point, w, h):
    scale = 0.2187609049 # Experimental calibration

    x = ( point[0] - ( w / 2.0 ) ) * scale
    y = ( point[1] - ( h / 2.0 ) ) * -1 * scale

    return [x, y]

total_area = w * h
area_max = 0.3
area_min = 0.003

kernel = np.ones((3,3),np.uint8)

path_offset = 10
path_spacing_kernel = np.ones((path_offset, path_offset), np.uint8)

mask_fill = np.zeros((h, w, 3), np.uint8)
mask_fill[:] = (0, 255, 0)

cv_file = cv2.FileStorage("CameraCalibrations/papalook_calibration.yaml", cv2.FILE_STORAGE_READ)

camera_matrix = cv_file.getNode('K').mat()
dist_matrix = cv_file.getNode('D').mat()

cv_file.release()

newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_matrix, (w,h), 0, (w,h))

while True:
    ret_val, original = cam.read()
#    original = cv2.imread("test.jpg")
    original = cv2.undistort(original, camera_matrix, dist_matrix, None, newcameramtx)

    mask = cv2.cvtColor(original, cv2.COLOR_BGR2GRAY)
    cv2.normalize(mask, mask, 0, 255, cv2.NORM_MINMAX)

    mask = cv2.threshold(mask, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]

    (numLabels, labels, stats, centroids) = cv2.connectedComponentsWithStats(mask)

    cMasks = {}
    for i in range(numLabels):
        componentMask = (labels == i).astype("uint8") * 255

        displayMask = cv2.bitwise_and(mask_fill, mask_fill, mask=componentMask)

        area = stats[i][3] * stats[i][2]
        r_area = area / total_area

        colour = (random.randint(0,255), random.randint(0,255), random.randint(0,255))

        if r_area < area_max and r_area > area_min:
            original = cv2.add(original, displayMask)

    cv2.imshow('cam', original)
    key = cv2.waitKey(1000)

    if key==27: ## esc
        break
    if key==32: ## space
        paths = []
        for i in range(numLabels):
            current_shape = []
            componentMask = (labels == i).astype("uint8") * 255

            area = stats[i][3] * stats[i][2]
            r_area = area / total_area

            colour = (random.randint(0,255), random.randint(0,255), random.randint(0,255))

            if r_area < area_max and r_area > area_min:
                while True:
                    cnts, hierarchy = cv2.findContours(componentMask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
                    if len(cnts) > 0:
                        current_shape.append(cnts)
                        cv2.drawContours(original, cnts, -1, colour, 2)
                    else:
                        break
                    componentMask = cv2.erode(componentMask, path_spacing_kernel)
                paths.append(current_shape)

        # Regions
            # Offset
                # Paths
                    # Points

        # Motion Type 0 - PTP, 1 - LIN
        # X in mm
        # Y in mm
        # Z in mm
        # rZ in degrees
        # rY in degrees (new y)
        # Travel speed in mm/s
        # Tool Number
        # Path Number

        robot_path = []
        pounce_height = 15.0
        draw_speed = 50.0
        tool_num = 1

        for region in paths:
            for offset in region:
                for path in offset:
                    if len(path) > 1: #Only if there is more than one point
                        point = pix2Robot(path[0][0], w, h)
                        robot_path.append([0,point[0],point[1],pounce_height,0.0,0.0,draw_speed,tool_num, tool_num])
                        for point_a in path:
                            point = pix2Robot(point_a[0], w, h)
                            robot_path.append([1,point[0],point[1],0.0,0.0,0.0,draw_speed,tool_num, tool_num])
                        point = pix2Robot(path[0][0], w, h)
                        robot_path.append([0,point[0],point[1],0.0,0.0,0.0,draw_speed,tool_num, tool_num])
                        robot_path.append([0,point[0],point[1],pounce_height,0.0,0.0,draw_speed,tool_num, tool_num])
            tool_num = tool_num + 1

        cv2.imshow('cam', original)
        cv2.waitKey(1000)

        Kuka_Src("config/kuka_config.yaml").writeScript("/home/davesarmoury/kuka_mount/colour.src",0,robot_path)

        break

cv2. destroyAllWindows()

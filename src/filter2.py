#!/usr/bin/env python3
import cv2
import numpy as np
import sys
import time
import math
from cv_bridge import CvBridge
from itertools import chain
import os

#import dictionary_msg
#globally declare a variable


x=0
value="plantType"
yaw=0
current_plant = 0
centroid_list={}
plants_to_look=[]
images_dict = {}
def crop(src):
    edges = cv2.Canny(src,50, 200, 10, 3)
    linesP = cv2.HoughLinesP(edges, 1, np.pi / 180, 50)
    # print(lines)
    x_max = 0
    x_min = 1000
    y_min = 1000

    if linesP is not None:
        for i in range(0, len(linesP)):
            if linesP is not None:
                for i in range(0, len(linesP)):
                    l = linesP[i][0]
                    for i in range(0, 4):
                        if i%2 == 0 :
                            if l[i]>x_max and l[i]>120:
                                x_max = l[i]
                            elif l[i]<x_min and l[i]>10 :
                                x_min = l[i]
                        else :
                            if l[i]<y_min and l[i]>98 :
                                y_min = l[i]
    cropped = src[0:y_min,x_min:x_max]
    return cropped
    # cv2.imshow("lines",src)
    # cv2.imshow("final",cropped)

def count(src):
    # src = crop(src)
    # cv2.imshow("cropped",src)
    if value == "Pepper" :
        color_lower_yellow = np.array([43,225,252])
        color_upper_yellow = np.array([166,254,255])
        mask_yellow = cv2.inRange(src, color_lower_yellow, color_upper_yellow)
        union_mask = mask_yellow
    elif value == "Tomato" :
        color_lower_red = np.array([0,0,120])
        color_upper_red = np.array([80,80,255])
        mask_red = cv2.inRange(src, color_lower_red, color_upper_red)
        union_mask = mask_red
    elif value == "Eggplant" :
        color_lower_violet = np.array([140,20,40])
        color_upper_violet = np.array([255,80,150])
        mask_violet = cv2.inRange(src, color_lower_violet, color_upper_violet)
        union_mask = mask_violet
    # color_lower_yellow = np.array([43,225,252])
    # color_upper_yellow = np.array([166,254,255])
    # mask_yellow = cv2.inRange(src, color_lower_yellow, color_upper_yellow)

    # color_lower_red = np.array([0,0,120])
    # color_upper_red = np.array([80,80,255])
    # mask_red = cv2.inRange(src, color_lower_red, color_upper_red)

    # color_lower_violet = np.array([140,20,40])
    # color_upper_violet = np.array([255,80,150])
    # mask_violet = cv2.inRange(src, color_lower_violet, color_upper_violet)

    #union_mask = np.bitwise_or(np.bitwise_or(mask_yellow, mask_red), mask_violet)

    ksize = (3, 3) 
        
    dilated_image = cv2.dilate(union_mask,ksize,iterations=3)
    eroded_image = cv2.erode(dilated_image,ksize,iterations=3)
    eroded_image = cv2.erode(eroded_image,ksize,iterations=3)
    dilated_image = cv2.dilate(eroded_image,ksize,iterations=3)

    # cv2.imshow("mask", dilated_image)
    
    contours, _ = cv2.findContours(dilated_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filtered_contours = [contour for contour in contours if cv2.contourArea(contour) >= 50]


    cv2.drawContours(dilated_image, filtered_contours, -1, (0, 255, 0), 2)
    
    height, width = dilated_image.shape

    white_image = np.ones((height, width, 3), dtype=np.uint8) * 255

    gray_image = dilated_image

    _, binary_image = cv2.threshold(gray_image, 200, 255, cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    contour_image = white_image.copy()
    # cv2.drawContours(contour_image, contours, -1, (0, 255, 0), 2)
    # cv2.imshow('mai chutiya ', contour_image)
    for contour in filtered_contours:

        global x
        x+=1


    # cv2.imshow('Detected Objects', dilated_image)
    # print(x)  



    centroids = []
    for contour in filtered_contours:
            M = cv2.moments(contour)
            X = int(M["m10"]/M["m00"])
            Y = int(M["m01"]/M["m00"])
            centroids.append([X,Y])


    return centroids

def process_image(image):
    img = image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, thresholded = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU)
    kernel = np.ones((3,3), np.uint8)
    eroded_image = cv2.erode(thresholded, kernel, iterations=5)
    kernel = np.ones((3,3), np.uint8)
    dilated_image = cv2.dilate(eroded_image, kernel, iterations=3)
    non_white_pixels = np.column_stack(np.where(dilated_image == 255))  # Adjust condition as needed
    if non_white_pixels.size == 0:
        return 0, 0, img.shape[1], img.shape[0], count(img), img  


    leftmost = non_white_pixels[:, 1].min()
    topmost = non_white_pixels[:, 0].min()
    rightmost = non_white_pixels[:, 1].max()
    bottommost = non_white_pixels[:, 0].max()
    
    image = img[topmost:bottommost + 1, leftmost:rightmost + 1]
    if not os.path.exists("images"):
        os.makedirs("images")
    if not os.path.exists(f"images/{current_plant}/"):
        os.makedirs(f"images/{current_plant}/")
    cv2.imwrite('./images'+f'/{current_plant}'+ f'/{yaw}.jpg',img=image)
    hi = []
    hi=count(image)
    return leftmost, topmost, rightmost, bottommost, hi, image
    
def callback(image):
    global current_plant,centroid_list,yaw
    left, top, right, bottom, hi, image_whore = process_image(image)
    if yaw == 1.5707:
        centroid_list[current_plant] = hi
    else:
        # print(len(hi[0]))
        for centroid in range(len(hi)):
            hi[centroid][0]=images_dict[current_plant][0].shape[1]*int(image.shape[1]-hi[centroid][0])//image.shape[1] #when looking from back
            hi[centroid][1] = (images_dict[current_plant][0].shape[0]*hi[centroid][0])//image.shape[0]
            # print(hi[0][centroid])
        centroid_list[current_plant]=centroid_list[current_plant]+hi
    # print(centroid_list[0])

          
def get_centroid_list(images,value_plant):
    global images_dict
    images_dict = images
    global current_plant,value,yaw
    value = value_plant
    #modify to include only active plants
    # cv2.imshow("yo",images[0][0])
    # cv2.imshow("yoyo",images[0][1])
    for i in images:
            # print("current plant is ",i)
            current_plant = i
            yaw = 1.5707
            callback(images[i][0])
            yaw = -1.5707
            callback(images[i][1])
    # print("len of centroid list is")
    # print(len(centroid_list[0]))
    # start = time.time()
    # crop(image_path)
    # left, top, right, bottom, hi, image_whore = process_image(image_path)
    # # print(hi)
    # cv2.imshow("Detected Fruits", image_whore)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
            #yo
    return centroid_list
    #call filter2 main with image dictionary and value_plant as shown on plant_beds example Pepper etc.
    
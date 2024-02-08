#!/usr/bin/env python3
import cv2
import numpy as np
import sys
import time
#globally declare a variable
x=0

def count(src):
    # if color=="yellow":   

        color_lower_yellow = np.array([43,225,252])
        color_upper_yellow = np.array([166,254,255])
        mask_yellow = cv2.inRange(src, color_lower_yellow, color_upper_yellow)
        # cv2.imshow("mask", mask_yellow)
        # print("teri maa ki chut peeli")
    # if color=="red":
        color_lower_red = np.array([0,0,120])
        color_upper_red = np.array([80,80,255])
        mask_red = cv2.inRange(src, color_lower_red, color_upper_red)
        # cv2.imshow("mask", mask_red)
        # print("teri maa ki chut laal")
    # if color=="violet":
        color_lower_violet = np.array([140,20,40])
        color_upper_violet = np.array([255,80,150])
        mask_violet = cv2.inRange(src, color_lower_violet, color_upper_violet)
        # cv2.imshow("mask", mask_violet)
        # print("teri maa ki chut baingani")
    # else :
    #     print("teri maa ki chut")
    #     centroids = []
    #     return centroids
        union_mask = np.bitwise_or(np.bitwise_or(mask_yellow, mask_red), mask_violet)
        #print union_mask as image
        #apply gaussian blur on image
        ksize = (5, 5) 

        blurred_image = cv2.GaussianBlur(union_mask, ksize, 10)
        
        # cv2.imshow("mask", union_mask)
        cv2.imshow("mask", blurred_image)
        # ! for contours
        # *
        contours, _ = cv2.findContours(blurred_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)



        filtered_contours = [contour for contour in contours if cv2.contourArea(contour) >= 200]
        # print(filtered_contours)

        #draw contours
        cv2.drawContours(blurred_image, filtered_contours, -1, (0, 255, 0), 2)
        #print(f'Number of Contours: {len(contours)}')
# Iterate through contours
        height, width = blurred_image.shape

# Create a white image with the same dimensions
        white_image = np.ones((height, width, 3), dtype=np.uint8) * 255

# Convert the image to grayscale
        gray_image = blurred_image

# Thresholding
        _, binary_image = cv2.threshold(gray_image, 200, 255, cv2.THRESH_BINARY)

# Find contours
        contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Draw contours on the white image
        contour_image = white_image.copy()
        cv2.drawContours(contour_image, contours, -1, (0, 255, 0), 2)
        cv2.imshow('mai chutiya ', contour_image)
        for contour in filtered_contours:
    # Filter contours based on area or other criteria
            # x+=1
            # print('here')
            global x
            x+=1
            #incremenet x by 1
            
            # area = cv2.contourArea(contour)
            # if area > 00 and area < 10:

        # Draw bounding box
            # x, y, w, h = cv2.boundingRect(contour)
            # blurred_image=cv2.rectangle(blurred_image, (x, y), (x + w, y + h), (0, 0, 255), 8)

# Display the result
        cv2.imshow('Detected Objects', blurred_image)
        print(x)
#         contours, _ = cv2.findContours(blurred_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#         # Print the number of contours
#         print(f'Number of Contours: {len(contours)}')

# # Iterate through contours
#         for contour in contours:
#     # Filter contours based on area or other criteria
#             area = cv2.contourArea(contour)
#             if area > 15:
#         # Draw bounding box
#                 x, y, w, h = cv2.boundingRect(contour)
#                 cv2.rectangle(blurred_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

# # Display the result
#         circles = cv2.HoughCircles(blurred_image, cv2.HOUGH_GRADIENT, dp=1, minDist=20, param1=50, param2=30, minRadius=5, maxRadius=50)

#         if circles is not None:
#             circles = np.uint16(np.around(circles))

#     # Print the number of circles
#             print(f'Number of Circles: {circles.shape[1]}')

#     # Draw the circles
#         for i in circles[0, :]:
#             cv2.circle(blurred_image, (i[0], i[1]), i[2], (0, 255, 0), 2)  # Draw the outer circle
#             cv2.circle(blurred_image, (i[0], i[1]), 2, (0, 0, 255), 3)  # Draw the center of the circle

# # Display the result
#         cv2.imshow('Detected Circles', blurred_image)
# # # Use Hough Ellipse Transform to detect ellipses
#         ellipses = cv2.fitEllipse(np.argwhere(blurred_image == 255))

# # Draw the circles
#         if circles is not None:
#             circles = np.uint16(np.around(circles))
#             for i in circles[0, :]:
#                 cv2.circle(blurred_image, (i[0], i[1]), i[2], (0, 255, 0), 2)  # Draw the outer circle
#                 cv2.circle(blurred_image, (i[0], i[1]), 2, (0, 0, 255), 3)  # Draw the center of the circle

# # Draw the ellipses
#             cv2.ellipse(blurred_image, ellipses, (0, 255, 0), 2)

# # Display the result
#         cv2.imshow('Detected Shapes', blurred_image)
    
    
    
    
    # contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        centroids = []

    # for contour in contours:
    #     area = cv2.contourArea(contour)
    #     if area > 1000:
    #         cv2.drawContours(src, [contour], 0, (25, 255, 0), cv2.FILLED)
    #         M = cv2.moments(contour)
    #         if M["m00"] != 0:
    #             cX = int(M["m10"] / M["m00"])
    #             cY = int(M["m01"] / M["m00"])
    #             # cv2.circle(src, (cX, cY), 5, (255, 255, 255), -1)
    #             centroids.append((cX, cY))

        return centroids

def process_image(image_path):
    img = cv2.imread(image_path)
    non_white_pixels = np.column_stack(np.where(np.all(img > [235, 235, 235], axis=-1)))

    leftmost = non_white_pixels[:, 1].min()
    topmost = non_white_pixels[:, 0].min()
    rightmost = non_white_pixels[:, 1].max()
    bottommost = non_white_pixels[:, 0].max()
    
    image = img[topmost:bottommost + 1, leftmost:rightmost + 1]
    # cv2.circle(image, (leftmost,topmost), 5, (255, 0 ,0), 2)
    hi = []
    hi.append(count(image))

    # hi.append(count(image, "red"))

    # hi.append(count(image, "violet"))

    # Return the bounding box coordinates along with centroids
    
    return leftmost, topmost, rightmost, bottommost, hi, image
    

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python fruit.py image_path ")
        sys.exit(1)

    start = time.time()
    image_path = sys.argv[1]
    
    left, top, right, bottom, hi, image_whore = process_image(image_path)
    print(hi)
    
    # print("yellow: ", len(hi[0]))
    # print("red: ", len(hi[1]))
    # print("violet: ", len(hi[2]))

    # for i, centroid in enumerate(hi[0]):
    #     x, y = centroid
    #     print(f"Yellow Centroid {i + 1}: x={x}, y={y}")
    # for i, centroid in enumerate(hi[1]):
    #     x, y = centroid
    #     print(f"Red Centroid {i + 1}: x={x}, y={y}")
    # for i, centroid in enumerate(hi[2]):
    #     x, y = centroid
    #     print(f"Violet Centroid {i + 1}: x={x}, y={y}")

    # Draw bounding box around each detected fruit
    for i, centroids in enumerate(hi):
        for centroid in centroids:
            x, y = centroid
            x += left  # Adjust x coordinate based on the region of interest
            y += top   # Adjust y coordinate based on the region of interest
            cv2.rectangle(image_whore, (x - 5, y - 5), (x + 5, y + 5), (0, 255, 0), 2)

    # Display the image with bounding boxes
    cv2.imshow("Detected Fruits", image_whore)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # end = time.time()

    # print(end - start)
